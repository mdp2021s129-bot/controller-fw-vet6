#![no_std]
#![no_main]

pub mod hdcomm;
// Panic handler, logging, all those nice things.
use controller_fw as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, USBWakeup, FSMC, SDIO, CAN_RX1, CAN_SCE, USB_HP_CAN_TX, USB_LP_CAN_RX0])]
mod app {
    use crate::hdcomm::{
        self, dh_tx, hd_rx, hd_rx_poll, DhDmaState, HdRxQueueConsumer, HdRxQueueProducer, RxCircDma,
    };
    use controller_fw::board::analog::Analog;
    use controller_fw::board::clock::RTICMonotonic;
    use controller_fw::board::startup::{self, DhTx, Steering, Wheels};
    use cortex_m::singleton;
    use stm32f1xx_hal::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = RTICMonotonic;

    #[shared]
    struct Shared {
        wheels: Wheels,
        steering: Steering,
        analog: Analog,
    }

    #[local]
    struct Local {
        /// Device to Host DMA state.
        dh_tx: DhDmaState<DhTx>,
        /// Host to Device continuous DMA transfer.
        hd_rx: RxCircDma,

        /// Host to device message queue (ISR -> idle)
        hd_rx_consumer: HdRxQueueConsumer,
        /// Host to device message queue (ISR -> idle)
        hd_rx_producer: HdRxQueueProducer,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (wheels, steering, analog, dh_tx, hd_rx, monotonic) =
            startup::startup(cx.core, cx.device);
        let hd_rx_pair: (HdRxQueueProducer, HdRxQueueConsumer) =
            hdcomm::receive_queue_split().unwrap();

        hdcomm::dh_rx_poll_start();

        defmt::info!("init complete");

        (
            Shared {
                wheels,
                steering,
                analog,
            },
            Local {
                dh_tx: DhDmaState::Idle(
                    dh_tx,
                    singleton!(: crate::hdcomm::TxBuf = heapless::Vec::new()).unwrap(),
                ),
                hd_rx: hd_rx
                    .circ_read_granular(singleton!(: crate::hdcomm::RxBuf = [0; 516]).unwrap()),
                hd_rx_consumer: hd_rx_pair.1,
                hd_rx_producer: hd_rx_pair.0,
            },
            init::Monotonics(monotonic),
        )
    }

    #[idle(local = [hd_rx_consumer])]
    fn idle(_: idle::Context) -> ! {
        loop {
            // TODO: Port over & clean up prototype message receive state machine.
        }
    }

    // Out-of-module tasks.
    extern "Rust" {
        #[task(binds = DMA1_CHANNEL4, local = [dh_tx])]
        fn dh_tx(_: dh_tx::Context);

        #[task(binds = DMA1_CHANNEL5, local = [hd_rx, hd_rx_producer, accum: hdcomm_device::Accumulator = hdcomm_device::Accumulator::new()])]
        fn hd_rx(_: hd_rx::Context);

        #[task()]
        fn hd_rx_poll(_: hd_rx_poll::Context);
    }
}
