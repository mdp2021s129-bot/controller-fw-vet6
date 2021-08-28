/// Board startup routines.
use super::clock;
use crate::board::analog::Analog;
use controller_core::board::lrtimer::LrTimer;
use controller_core::board::motion::{Steering as GenericSteering, Wheels as GenericWheels};
use cortex_m::Peripherals as CorePeripherals;
use stm32f1xx_hal::{
    dma,
    gpio::{Alternate, ErasedPin, Floating, Input, Output, PushPull},
    pac,
    prelude::*,
    pwm::Channel,
    qei::QeiOptions,
    serial, timer,
};

pub type Wheels = GenericWheels<
    stm32f1xx_hal::pwm::Pwm<
        stm32f1xx_hal::pac::TIM8,
        timer::Tim8NoRemap,
        (stm32f1xx_hal::pwm::C2, stm32f1xx_hal::pwm::C3),
        (
            stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRL, 'C', 7_u8>,
            stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRH, 'C', 8_u8>,
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
    stm32f1xx_hal::qei::Qei<
        stm32f1xx_hal::pac::TIM4,
        timer::Tim4NoRemap,
        (
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRL, 'B', 6_u8>,
            stm32f1xx_hal::gpio::Pin<Input<Floating>, stm32f1xx_hal::gpio::CRL, 'B', 7_u8>,
        ),
    >,
    ErasedPin<Output<PushPull>>,
>;

pub type Steering = GenericSteering<
    stm32f1xx_hal::pwm::Pwm<
        stm32f1xx_hal::pac::TIM1,
        timer::Tim1FullRemap,
        stm32f1xx_hal::pwm::C2,
        stm32f1xx_hal::gpio::Pin<Alternate<PushPull>, stm32f1xx_hal::gpio::CRH, 'E', 11_u8>,
    >,
>;

pub type DhTx = dma::TxDma<serial::Tx<stm32f1xx_hal::pac::USART3>, stm32f1xx_hal::dma::dma1::C2>;
pub type HdRx = dma::RxDma<serial::Rx<stm32f1xx_hal::pac::USART3>, stm32f1xx_hal::dma::dma1::C3>;

/// Starts up the board, returning resources provided by it.
pub fn startup(
    mut core: CorePeripherals,
    periph: pac::Peripherals,
) -> (
    Wheels,
    Steering,
    Analog,
    DhTx,
    HdRx,
    LrTimer,
    clock::RTICMonotonic,
) {
    let mut afio = periph.AFIO.constrain();
    let flash = periph.FLASH.constrain();
    let rcc = periph.RCC.constrain();

    // Clock tree initialization
    let clocks = clock::clock_tree_setup(flash.acr, rcc);

    // GPIO initialization
    let mut gpioa = periph.GPIOA.split();
    let mut gpiob = periph.GPIOB.split();
    let mut gpioc = periph.GPIOC.split();
    let mut gpioe = periph.GPIOE.split();

    // DMA1 initialization
    let dma1 = periph.DMA1.split();

    // Motor control initialization
    let wheels = {
        let in1_l = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let in2_l = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

        let in1_r = gpioe.pe12.into_push_pull_output(&mut gpioe.crh);
        let in2_r = gpioc.pc5.into_push_pull_output(&mut gpioc.crl);

        let pwm_l = gpioc.pc7.into_alternate_push_pull(&mut gpioc.crl);
        let pwm_r = gpioc.pc8.into_alternate_push_pull(&mut gpioc.crh);
        let pwm = timer::Timer::tim8(periph.TIM8, &clocks).pwm((pwm_l, pwm_r), 20.khz());

        afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let enc_l = {
            let enca_l = gpioa.pa6.into_floating_input(&mut gpioa.crl);
            let encb_l = gpioa.pa7.into_floating_input(&mut gpioa.crl);
            timer::Timer::tim3(periph.TIM3, &clocks).qei(
                (enca_l, encb_l),
                &mut afio.mapr,
                QeiOptions {
                    slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
                    ..Default::default()
                },
            )
        };

        let enc_r = {
            let enca_r = gpiob.pb6.into_floating_input(&mut gpiob.crl);
            let encb_r = gpiob.pb7.into_floating_input(&mut gpiob.crl);
            timer::Timer::tim4(periph.TIM4, &clocks).qei(
                (enca_r, encb_r),
                &mut afio.mapr,
                QeiOptions {
                    slave_mode: stm32f1xx_hal::qei::SlaveMode::EncoderMode3,
                    ..Default::default()
                },
            )
        };

        Wheels::new(
            pwm,
            40000.hz(),
            [
                [in1_l.erase(), in2_l.erase()],
                [in1_r.erase(), in2_r.erase()],
            ],
            [Channel::C2, Channel::C3],
            (enc_l, enc_r),
        )
    };

    // Steering initialization.
    let steering = {
        let servo_ctrl = gpioe.pe11.into_alternate_push_pull(&mut gpioe.crh);
        let servo_ctrl_pwm =
            timer::Timer::tim1(periph.TIM1, &clocks).pwm(servo_ctrl, &mut afio.mapr, 50.hz());
        Steering::new(servo_ctrl_pwm, Channel::C2)
    };

    // Analog sensors initialization.
    let analog = {
        let vin = gpioc.pc4.into_analog(&mut gpioc.crl);

        Analog::new(vin, periph.ADC1, clocks)
    };

    // D -> H UART initialization.
    let (dh_tx, hd_rx) = {
        let tx = gpioc.pc10.into_alternate_push_pull(&mut gpioc.crh);
        let rx = gpioc.pc11.into_floating_input(&mut gpioc.crh);
        let dh_periph = serial::Serial::usart3(
            periph.USART3,
            (tx, rx),
            &mut afio.mapr,
            serial::Config {
                baudrate: 115200.bps(),
                parity: serial::Parity::ParityNone,
                stopbits: serial::StopBits::STOP1,
            },
            clocks,
        );
        let (dh_tx_periph, hd_rx_periph) = dh_periph.split();

        let mut dh_tx = dh_tx_periph.with_dma(dma1.2);
        dh_tx.channel.listen(dma::Event::TransferComplete);

        let mut hd_rx = hd_rx_periph.with_dma(dma1.3);
        hd_rx.channel.listen(dma::Event::HalfTransfer);
        hd_rx.channel.listen(dma::Event::TransferComplete);

        (dh_tx, hd_rx)
    };

    // LrTimer initialization.
    let lrtimer = LrTimer::new(periph.TIM2, &clocks);

    // Monotonic clock initialization.
    let monotonic = clock::monotonic_setup(&mut core.DCB, core.DWT, core.SYST, clocks.sysclk().0);
    (wheels, steering, analog, dh_tx, hd_rx, lrtimer, monotonic)
}
