#![no_std]
#![no_main]

pub mod hdcomm;
pub mod trajectory;
// Panic handler, logging, all those nice things.
use controller_fw as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, USBWakeup, FSMC, SDIO, CAN_RX1, CAN_SCE, USB_HP_CAN_TX, USB_LP_CAN_RX0])]
mod app {
    use crate::hdcomm::{
        self, dh_tx, enqueue_device_to_host, hd_rx, hd_rx_poll, DhDmaState, HdRxQueueConsumer,
        HdRxQueueProducer, RxCircDma,
    };
    use crate::trajectory::{Controller, ControllerContext, Events as ControllerEvent};
    use controller_core::board::lrtimer::LrTimer;
    use controller_fw::board::analog::Analog;
    use controller_fw::board::clock::RTICMonotonic;
    use controller_fw::board::startup::{self, Ahrs, DhTx};
    use cortex_m::singleton;
    use hdcomm_core::rpc::{MoveRepBody, PidParamUpdateReqBody, PidParams};
    use stm32f1xx_hal::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = RTICMonotonic;

    #[shared]
    struct Shared {
        analog: Analog,
        lrtimer: LrTimer,
        trajectory_controller: Controller,
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

        /// Onboard MPU9250 AHRS.
        ahrs: Ahrs,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (wheels, steering, mut analog, dh_tx, hd_rx, ahrs, lrtimer, monotonic) =
            startup::startup(cx.core, cx.device);

        let hd_rx_pair: (HdRxQueueProducer, HdRxQueueConsumer) =
            hdcomm::receive_queue_split().unwrap();
        hdcomm::dh_rx_poll_start();

        let trajectory_controller = {
            let pid_params = PidParams {
                kp: 0.01,
                ki: 0.00004,
                kd: 0.01,
                p_limit: 1.,
                i_limit: 1.,
                d_limit: 0.,
                output_limit: 0.,
            };
            let ctrl_context = ControllerContext::new(
                &PidParamUpdateReqBody {
                    params: [pid_params.clone(), pid_params],
                    update_interval_ms: 10,
                },
                wheels,
                steering,
            );
            Controller::new(ctrl_context)
        };

        defmt::info!("init complete");
        (
            Shared {
                analog,
                lrtimer,
                trajectory_controller,
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
                ahrs,
            },
            init::Monotonics(monotonic),
        )
    }

    #[idle(local = [hd_rx_consumer], shared = [trajectory_controller, lrtimer])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            if let Some(msg) = cx.local.hd_rx_consumer.dequeue() {
                use hdcomm_core::message::{Message, Payload};
                use hdcomm_core::rpc::{Message as RPCMessage, Payload as RPCPayload};
                // Annotate for rust-analyzer.
                let msg: Message = msg;
                let rpc = match msg.payload {
                    Payload::RPC(r) => r,
                    _ => {
                        defmt::warn!("Non-RPC message received");
                        continue;
                    }
                };

                let id = rpc.id;
                defmt::info!("HD RX: RPC ID {}", id);
                let reply_payload = match rpc.payload {
                    RPCPayload::PingReq(_) => Some(RPCPayload::PingRep(())),
                    RPCPayload::MoveReq(mv) => {
                        let now = cx.shared.lrtimer.lock(|t| t.now());
                        Some(
                            if cx
                                .shared
                                .trajectory_controller
                                .lock(|c| c.move_request(mv, now))
                            {
                                RPCPayload::MoveRep(MoveRepBody::Accepted)
                            } else {
                                RPCPayload::MoveRep(MoveRepBody::Busy)
                            },
                        )
                    }
                    _ => None,
                };

                if let Some(rpayload) = reply_payload {
                    let reply = Message {
                        payload: Payload::RPC(RPCMessage {
                            id,
                            payload: rpayload,
                        }),
                    };

                    if enqueue_device_to_host(reply).is_err() {
                        defmt::warn!("DH TX: queue full: dropped")
                    }
                }
            }
        }
    }

    // Capacity of 2 to hold at least one cancel request.
    // TODO: use new RTIC .cancel API
    // TODO: migrate to separate module
    #[task(shared = [trajectory_controller], capacity = 2)]
    fn trajectory_controller_service(
        mut cx: trajectory_controller_service::Context,
        event: ControllerEvent,
    ) {
        let now = monotonics::MyMono::now();
        match cx.shared.trajectory_controller.lock(|c| c.service(event)) {
            Some((event, after)) => {
                if trajectory_controller_service::spawn_at(now + after, event).is_err() {
                    panic!("trajectory_controller_service overflow")
                }
            }
            None => (),
        };
    }

    /// LrTimer overflow update handler.
    #[task(binds = TIM2, shared = [lrtimer])]
    fn lrtimer_update(mut cx: lrtimer_update::Context) {
        cx.shared.lrtimer.lock(|t| t.isr());
    }

    // Out-of-module tasks.
    extern "Rust" {
        #[task(binds = DMA1_CHANNEL2, local = [dh_tx])]
        fn dh_tx(_: dh_tx::Context);

        #[task(binds = DMA1_CHANNEL3, local = [hd_rx, hd_rx_producer, accum: hdcomm_device::Accumulator = hdcomm_device::Accumulator::new()])]
        fn hd_rx(_: hd_rx::Context);

        #[task()]
        fn hd_rx_poll(_: hd_rx_poll::Context);
    }
}
