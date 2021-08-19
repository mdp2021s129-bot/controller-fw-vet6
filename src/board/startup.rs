/// Board startup routines.
use super::clock;
use controller_core::board::motion::{Steering, Wheels};
use cortex_m::Peripherals as CorePeripherals;
use stm32f1xx_hal::{
    gpio::{Alternate, ErasedPin, Floating, Input, Output, PushPull},
    pac,
    prelude::*,
    pwm::Channel,
    qei::QeiOptions,
    timer,
};

pub type WheelsType = Wheels<
    stm32f1xx_hal::pwm::Pwm<
        stm32f1xx_hal::pac::TIM8,
        timer::Tim8NoRemap,
        (stm32f1xx_hal::pwm::C1, stm32f1xx_hal::pwm::C2),
        (
            stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRL, 'C', 6_u8>,
            stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRL, 'C', 7_u8>,
        ),
    >,
    stm32f1xx_hal::qei::Qei<
        stm32f1xx_hal::pac::TIM2,
        timer::Tim2PartialRemap1,
        (
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRH, 'A', 15_u8>,
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRL, 'B', 3_u8>,
        ),
    >,
    stm32f1xx_hal::qei::Qei<
        stm32f1xx_hal::pac::TIM3,
        timer::Tim3NoRemap,
        (
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRL, 'A', 6_u8>,
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRL, 'A', 7_u8>,
        ),
    >,
    ErasedPin<Output<PushPull>>,
>;

pub type SteeringType = Steering<
    stm32f1xx_hal::pwm::Pwm<
        stm32f1xx_hal::pac::TIM4,
        timer::Tim4Remap,
        stm32f1xx_hal::pwm::C1,
        stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRH, 'D', 12_u8>,
    >,
>;

/// Starts up the board, returning resources provided by it.
pub fn startup(
    mut core: CorePeripherals,
    periph: pac::Peripherals,
) -> (WheelsType, SteeringType, clock::RTICMonotonic) {
    let mut afio = periph.AFIO.constrain();
    let flash = periph.FLASH.constrain();
    let rcc = periph.RCC.constrain();

    // Clock tree initialization
    let clocks = clock::clock_tree_setup(flash.acr, rcc);

    // GPIO initialization
    let mut gpioa = periph.GPIOA.split();
    let gpiob = periph.GPIOB.split();
    let mut gpioc = periph.GPIOC.split();
    let mut gpiod = periph.GPIOD.split();

    // Motor control initialization
    let in1_l = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let in2_l = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

    let in1_r = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let in2_r = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

    let pwm_l = gpioc.pc6.into_alternate_push_pull(&mut gpioc.crl);
    let pwm_r = gpioc.pc7.into_alternate_push_pull(&mut gpioc.crl);
    let pwm = timer::Timer::tim8(periph.TIM8, &clocks).pwm((pwm_l, pwm_r), 20.khz());

    let (enca_l, encb_l, _) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    let enca_r = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let encb_r = gpioa.pa7.into_floating_input(&mut gpioa.crl);

    let enc_l = timer::Timer::tim2(periph.TIM2, &clocks).qei(
        (enca_l, encb_l),
        &mut afio.mapr,
        QeiOptions {
            slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
            ..Default::default()
        },
    );

    let enc_r = timer::Timer::tim3(periph.TIM3, &clocks).qei(
        (enca_r, encb_r),
        &mut afio.mapr,
        QeiOptions {
            slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
            ..Default::default()
        },
    );

    let wheels = Wheels::new(
        pwm,
        40000.hz(),
        [
            [in1_l.erase(), in2_l.erase()],
            [in1_r.erase(), in2_r.erase()],
        ],
        [Channel::C1, Channel::C2],
        (enc_l, enc_r),
    );

    // Steering initialization.
    let servo_ctrl = gpiod.pd12.into_alternate_push_pull(&mut gpiod.crh);
    let servo_ctrl_pwm =
        timer::Timer::tim4(periph.TIM4, &clocks).pwm(servo_ctrl, &mut afio.mapr, 100.hz());
    let steering = Steering::new(servo_ctrl_pwm, Channel::C1);

    // Monotonic clock initialization.
    let monotonic = clock::monotonic_setup(&mut core.DCB, core.DWT, core.SYST, clocks.sysclk().0);
    (wheels, steering, monotonic)
}
