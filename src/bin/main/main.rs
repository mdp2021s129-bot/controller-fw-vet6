#![no_std]
#![no_main]

pub mod hdcomm;
pub mod trajectory;
pub mod ultrasonic;
// Panic handler, logging, all those nice things.
use controller_fw as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, FSMC, SDIO, CAN_RX1, CAN_SCE, USB_HP_CAN_TX, USB_LP_CAN_RX0])]
mod app {
    use crate::hdcomm::{
        self, dh_tx, enqueue_device_to_host, hd_rx, hd_rx_idle, DhDmaState, HdRxQueueConsumer,
        HdRxQueueProducer, RxCircDma,
    };
    use crate::trajectory::{CallbackKind, Controller, Status as ControllerStatus};
    use crate::ultrasonic::{Error as Sr04Error, Event as Sr04Event, Sr04};
    use controller_core::board::lrtimer::LrTimer;
    use controller_fw::board::analog::Analog;
    use controller_fw::board::clock::RTICMonotonic;
    use controller_fw::board::startup::{self, Ahrs, DhTx, Sr04EchoPin, Sr04TriggerPin};
    use cortex_m::singleton;
    use hdcomm_core::rpc::{
        MoveRepBody, MoveStatusRepBody, PidParamUpdateRepBody, PidParamUpdateReqBody, PidParams,
        PingRepBody, RawTeleOpRepBody,
    };
    use rtic::time::duration::Milliseconds;
    use stm32f1xx_hal::{gpio::ExtiPin, prelude::*};

    #[monotonic(binds = SysTick, default = true)]
    type Dwt = RTICMonotonic;

    #[shared]
    struct Shared {
        /// Shared low-resolution (1ms resolution) timer.
        lrtimer: LrTimer,
        /// Robot trajectory controller.
        trajectory_controller: Controller,
        // Ultrasonic distance sensor.
        front_distance: Sr04<Sr04TriggerPin>,
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
        /// Analog sensors.
        analog: Analog,

        // Ultrasonic distance sensor echo pin.
        front_distance_echo: Sr04EchoPin,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (
            wheels,
            steering,
            analog,
            front_distance_echo,
            front_distance_trig,
            dh_tx,
            hd_rx,
            ahrs,
            lrtimer,
            monotonic,
        ) = startup::startup(cx.core, cx.device);

        let hd_rx_pair: (HdRxQueueProducer, HdRxQueueConsumer) =
            hdcomm::receive_queue_split().unwrap();

        // Trajectory controller init.
        let trajectory_controller = {
            // These parameters are all-zeroes.
            // The PID parameters must be updated by the host AP.
            let initial_params = PidParamUpdateReqBody {
                params: [PidParams::default(), PidParams::default()],
                update_interval_ms: 10,
            };
            Controller::new(&initial_params, wheels, steering)
        };

        // Front distance sensor init.
        let front_distance = Sr04::new(front_distance_trig);
        front_sensor_auto_trigger::spawn().unwrap();

        // sensor_dump_test::spawn_after(Milliseconds(1000_u32)).unwrap();

        defmt::info!("init complete");
        (
            Shared {
                lrtimer,
                trajectory_controller,
                front_distance,
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
                analog,
                front_distance_echo,
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
                let reply_payload = match rpc.payload {
                    RPCPayload::PingReq(_) => Some(RPCPayload::PingRep(PingRepBody {
                        time_ms: cx.shared.lrtimer.lock(|t| t.ms()),
                    })),
                    RPCPayload::MoveReq(mv) => {
                        let now = cx.shared.lrtimer.lock(|t| t.now());
                        Some(
                            if cx
                                .shared
                                .trajectory_controller
                                .lock(|c| c.request_move(mv, now))
                            {
                                RPCPayload::MoveRep(MoveRepBody::Accepted)
                            } else {
                                RPCPayload::MoveRep(MoveRepBody::Busy)
                            },
                        )
                    }
                    RPCPayload::MoveCancelReq(_) => {
                        cx.shared.trajectory_controller.lock(|c| c.cancel_move());
                        Some(RPCPayload::MoveCancelRep(()))
                    }
                    RPCPayload::MoveStatusReq(_) => {
                        let now = cx.shared.lrtimer.lock(|t| t.now());
                        let body = match cx.shared.trajectory_controller.lock(|c| c.status()) {
                            ControllerStatus::Idle => MoveStatusRepBody::NoCommand,
                            ControllerStatus::Active { start, required } => {
                                let elapsed = now - start;
                                let elapsed = (elapsed.integer() as f32)
                                    * (*elapsed.scaling_factor().numerator() as f32
                                        / *elapsed.scaling_factor().denominator() as f32);
                                MoveStatusRepBody::Executing {
                                    elapsed,
                                    remaining: required - elapsed,
                                }
                            }
                        };

                        Some(RPCPayload::MoveStatusRep(body))
                    }
                    RPCPayload::PidParamUpdateReq(params) => Some(RPCPayload::PidParamUpdateRep(
                        if cx
                            .shared
                            .trajectory_controller
                            .lock(|c| c.set_pid_parameters(&params))
                        {
                            PidParamUpdateRepBody::Updated
                        } else {
                            PidParamUpdateRepBody::Busy
                        },
                    )),
                    RPCPayload::RawTeleOpReq(req) => Some(RPCPayload::RawTeleOpRep(
                        if cx
                            .shared
                            .trajectory_controller
                            .lock(|c| c.raw_control(&req))
                        {
                            RawTeleOpRepBody::Applied
                        } else {
                            RawTeleOpRepBody::Busy
                        },
                    )),
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

    /// Task meant to call back into the trajectory controller.
    #[task(shared = [trajectory_controller], capacity = 2)]
    fn trajectory_controller_callback(
        mut cx: trajectory_controller_callback::Context,
        kind: CallbackKind,
    ) {
        cx.shared.trajectory_controller.lock(|c| c.callback(kind));
    }

    /// LrTimer overflow update handler.
    #[task(binds = TIM2, shared = [lrtimer])]
    fn lrtimer_update(mut cx: lrtimer_update::Context) {
        cx.shared.lrtimer.lock(|t| t.isr());
    }

    /// Task meant to call back into the front ultrasonic sensor driver on
    /// trigger pin edges.
    ///
    /// Runs at high priority to avoid being preempted: we want to preserve high timing
    /// accuracy.
    #[task(binds = EXTI4, shared = [lrtimer, front_distance], local = [front_distance_echo], priority = 16)]
    fn front_distance_callback_trig(mut cx: front_distance_callback_trig::Context) {
        let lr_now = cx.shared.lrtimer.lock(|l| l.now());
        cx.local.front_distance_echo.clear_interrupt_pending_bit();
        if let Err(Sr04Error::Unexpected) = cx
            .shared
            .front_distance
            .lock(|d| d.callback(Sr04Event::EchoInterrupt, lr_now))
        {
            defmt::warn!("unexpected echo interrupt")
        }
    }

    /// Task meant to execute after sensor driver trigger delay completion.
    #[task(shared = [lrtimer, front_distance])]
    fn front_sensor_delay_callback(mut cx: front_sensor_delay_callback::Context) {
        let lr_now = cx.shared.lrtimer.lock(|l| l.now());
        if let Err(Sr04Error::Unexpected) = cx
            .shared
            .front_distance
            .lock(|d| d.callback(Sr04Event::TriggerComplete, lr_now))
        {
            defmt::warn!("unexpected trigger complete")
        }
    }

    /// Task meant to periodically trigger the ultrasonic sensor.
    #[task(shared = [lrtimer, front_distance])]
    fn front_sensor_auto_trigger(mut cx: front_sensor_auto_trigger::Context) {
        front_sensor_auto_trigger::spawn_after(Milliseconds(100_u32)).unwrap();

        let lr_now = cx.shared.lrtimer.lock(|l| l.now());
        if let Err(Sr04Error::InProgress) = cx.shared.front_distance.lock(|d| d.trigger(lr_now)) {
            defmt::warn!("measurement in progresss")
        }
    }

    /// AHRS streamer.
    ///
    /// Continually sends AHRS samples to the host.
    #[task(local = [ahrs], shared = [lrtimer])]
    fn ahrs_streamer(mut cx: ahrs_streamer::Context) {
        ahrs_streamer::spawn_after(Milliseconds(10_u32)).unwrap();

        let ahrs: &mut Ahrs = cx.local.ahrs;
        let readings: Result<mpu9250::UnscaledMargMeasurements<[i16; 3]>, _> = ahrs.unscaled_all();
        if let Ok(meas) = readings {
            use hdcomm_core::message::Message;
            use hdcomm_core::stream::{AhrsBody, Message as StreamMessage, Payload};

            let now = cx.shared.lrtimer.lock(|l| l.ms());
            let ahrs_sample = AhrsBody {
                acc: meas.accel,
                gyro: meas.gyro,
                mag: meas.mag,
                time_ms: now,
            };
            if let Err(_) = enqueue_device_to_host(Message {
                payload: hdcomm_core::message::Payload::Stream(StreamMessage {
                    payload: Payload::Ahrs(ahrs_sample),
                }),
            }) {
                defmt::warn!("DH TX queue full: AHRS sample dropped");
            }
        } else {
            defmt::warn!("error reading AHRS")
        }
    }

    // Out-of-module tasks.
    extern "Rust" {
        #[task(binds = DMA1_CHANNEL2, local = [dh_tx])]
        fn dh_tx(_: dh_tx::Context);

        #[task(binds = DMA1_CHANNEL3, local = [hd_rx, hd_rx_producer, accum: hdcomm_device::Accumulator = hdcomm_device::Accumulator::new()], priority = 15)]
        fn hd_rx(_: hd_rx::Context);

        #[task(binds = USART3, priority = 15)]
        fn hd_rx_idle(_: hd_rx_idle::Context);
    }
}
