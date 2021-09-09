/// Host <-> device communication implementation.
use controller_fw::board::startup::HdRx;
use core::ops::Deref;
use cortex_m::singleton;
use hdcomm_core::message::Message;
use stable_deref_trait::StableDeref;
use stm32f1xx_hal::{
    dma,
    pac::{self, Interrupt},
    prelude::*,
};

/// Type of the receive buffer, the area of memory that can hold at least one
/// (complete) incoming COBS-framed but not yet deserialized `Message`.
pub type RxBuf = [u8; hdcomm_device::ENCODED_BUFFER_SIZE * 2];

/// Type of the receive DMA operation.
pub type RxCircDma = dma::CircBufferGranular<&'static mut [u8], HdRx>;

/// Type of the transmit buffer, the area of memory that will be used to hold a
/// serialized but not yet transmitted `Message`.
pub type TxBuf = heapless::Vec<u8, { hdcomm_device::ENCODED_BUFFER_SIZE }>;

/// Reference to the transmit buffer.
///
/// `'static` for DMA.
pub type TxBufRef = &'static mut TxBuf;

/// Message receive queue type.
///
/// Used to funnel messages between the DMA RX handler & the idle task
/// responsible for replying to messages.
pub type HdRxQueue = heapless::spsc::Queue<Message, 8>;

/// Message receive queue producer type.
pub type HdRxQueueProducer = heapless::spsc::Producer<'static, Message, 8>;

/// Message receive queue consumer type.
pub type HdRxQueueConsumer = heapless::spsc::Consumer<'static, Message, 8>;

/// Message transmit queue.
static DH_TX_QUEUE: heapless::mpmc::MpMcQueue<Message, 8> = heapless::mpmc::MpMcQueue::new();

/// `TxBufRefWrapper` wraps a transmit buffer reference.
///
/// It exists so that we do not need to hold a second reference to the transmit
/// buffer while we send it off for DMA.
///
/// Passing a `&[u8]` to the DMA API causes us to lose access to the resizing
/// capabilities of the Vec unless we hold a reference to it, but that has its
/// own problems becuase then we can no longer hold a mutable reference after
/// passing a reference to the DMA API.
///
/// We use `TxBufRefWrapper` to pass the mutable reference **through** the DMA
/// API and have it returned back to us after the DMA is complete.
pub struct TxBufRefWrapper(pub TxBufRef);

impl Deref for TxBufRefWrapper {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.0.as_slice()
    }
}

impl TxBufRefWrapper {
    /// Retrieves the wrapped buffer reference.
    pub fn into_inner(self) -> TxBufRef {
        self.0
    }
}

/// Required for DMA.
///
// UNSAFE: should be sound because we are wrapping a 'static healpess Vec.
// The starting address of the Vec's memory block should not change, so the
// pointer yielded from `deref` must always be safe to access.
unsafe impl StableDeref for TxBufRefWrapper {}

/// State of the Device to Host DMA channel.
pub enum DhDmaState<PORT: dma::TransferPayload> {
    /// DMA is actively transferring data.
    Transferring(dma::Transfer<dma::R, TxBufRefWrapper, PORT>),
    /// DMA is idle.
    Idle(PORT, TxBufRef),
    /// Required because RTIC does not provide direct access to resources.
    ///
    /// We use `std::mem::swap` to avoid the need for an extra `Option`.
    Swapped,
}

/// Obtain handles to the receive queue.
///
/// Returns `None` if called more than once.
pub fn receive_queue_split() -> Option<(HdRxQueueProducer, HdRxQueueConsumer)> {
    let queue = singleton!(: HdRxQueue = HdRxQueue::new());
    queue.map(|q| q.split())
}

/// Queues a message for transmission to the host.
///
/// The message is returned if the transmission queue is full.
pub fn enqueue_device_to_host(message: Message) -> Result<(), Message> {
    match DH_TX_QUEUE.enqueue(message) {
        Ok(_) => {
            // Pend channel so that the buffer gets queued for transmission.
            rtic::pend(Interrupt::DMA1_CHANNEL2);
            Ok(())
        }
        Err(e) => Err(e),
    }
}

/// Task handling host-device transmission completion.
///
/// Will be called in these scenarios:
///
/// - A new buffer is queued for transmission.
/// - A buffer has completed transmission.
///
/// Serialization is performed in this task, so it doesn't have elevated
/// priority.
pub fn dh_tx(cx: crate::app::dh_tx::Context) {
    let mut dh_tx = DhDmaState::Swapped;
    core::mem::swap(cx.local.dh_tx, &mut dh_tx);

    dh_tx = match dh_tx {
        // We are idle, so it's definitely a new buffer being queued.
        DhDmaState::Idle(port, buf) => {
            let message = DH_TX_QUEUE.dequeue().unwrap();
            buf.clear();
            hdcomm_device::into_vec(&message, buf).unwrap();
            DhDmaState::Transferring(port.write(TxBufRefWrapper(buf)))
        }
        // We are transferring, so it could be: transfer completion OR a
        // new buffer being queued.
        DhDmaState::Transferring(t) => {
            // Do nothing if the transfer is not complete.
            if t.is_done() {
                // Transfer complete.
                let (buf_wrapper, port) = t.wait();
                let buf = buf_wrapper.into_inner();

                // Queue up another transfer if we have messages in the buffer.
                match DH_TX_QUEUE.dequeue() {
                    Some(message) => {
                        buf.clear();
                        hdcomm_device::into_vec(&message, buf).unwrap();
                        DhDmaState::Transferring(port.write(TxBufRefWrapper(buf)))
                    }
                    None => DhDmaState::Idle(port, buf),
                }
            } else {
                defmt::warn!("DH TX DMA busy: queued");
                DhDmaState::Transferring(t)
            }
        }
        DhDmaState::Swapped => unreachable!(),
    };

    core::mem::swap(cx.local.dh_tx, &mut dh_tx);
}

/// Task handling device-host reception.
///
/// Will be called in the following scenarios:
///
/// - DMA circular transfer pointer wrapped around end of buffer.
/// - DMA circular transfer pointer crossed half of the buffer.
/// - After every line idle interrupt.
///
/// Deserialization is also performed within this task, so it doesn't
/// have elevated priority.
pub fn hd_rx(cx: crate::app::hd_rx::Context) {
    let mut buffer: crate::hdcomm::RxBuf = [0; 516];
    let buffer = &mut buffer[..];

    let read = cx.local.hd_rx.read(buffer);
    let mut window = &buffer[..read];

    // Drops are _not_ errors.
    use postcard::FeedResult;
    'cobs: while !window.is_empty() {
        window = match cx.local.accum.feed(window) {
            FeedResult::Consumed => break 'cobs,
            FeedResult::OverFull(new_wind) => {
                defmt::warn!("HD RX DMA: COBS overflow: dropped");
                new_wind
            }
            FeedResult::DeserError(new_wind) => {
                defmt::warn!("HD RX DMA: deser error: dropped");
                new_wind
            }
            FeedResult::Success { data, remaining } => {
                if cx.local.hd_rx_producer.enqueue(data).is_err() {
                    defmt::warn!("HD RX DMA: queue full: dropped");
                }
                remaining
            }
        };
    }
}

/// Host -> device receive idle handler.
///
/// Simply pends the `hd_rx` handler to handle the incoming message.
pub fn hd_rx_idle(_: crate::app::hd_rx_idle::Context) {
    // No safe API to clear idle interrupt flag while DMA is ongoing.
    unsafe {
        let usart3 = pac::Peripherals::steal().USART3;
        // Read registers as specified in the RM to clear the idle interrupt
        // flag.
        usart3.sr.read();
        usart3.dr.read();
    }

    // Pend the host -> device receive handler to process the burst.
    rtic::pend(Interrupt::DMA1_CHANNEL3);
}
