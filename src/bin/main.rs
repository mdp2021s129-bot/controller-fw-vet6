#![deny(unsafe_code)]
#![no_std]
#![no_main]

// Panic handler, logging, all those nice things.
use controller_fw as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, USBWakeup, FSMC, SDIO, CAN_RX1, CAN_SCE, USB_HP_CAN_TX, USB_LP_CAN_RX0])]
mod app {
    use controller_fw::board::clock::RTICMonotonic;
    use controller_fw::board::startup::{self, Steering, Wheels};
    use rtic::rtic_monotonic::Milliseconds;
    use stm32f1xx_hal::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = RTICMonotonic;

    #[shared]
    struct Shared {
        wheels: Wheels,
        steering: Steering,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (wheels, steering, monotonic) = startup::startup(cx.core, cx.device);
        defmt::info!("init complete");

        (
            Shared { wheels, steering },
            Local {},
            init::Monotonics(monotonic),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }
}
