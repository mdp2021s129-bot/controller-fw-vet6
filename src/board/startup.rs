/// Board startup routines.
use super::clock;
use crate::board::analog::Analog;
use controller_core::board::lrtimer::LrTimer;
use controller_core::board::motion::{Steering as GenericSteering, Wheels as GenericWheels};
use cortex_m::Peripherals as CorePeripherals;
use stm32f1xx_hal::{
    dma,
    gpio::{
        Alternate, ErasedPin, ExtiPin, Floating, Input, OpenDrain, Output, Pin, PinState, PushPull,
        CRH, CRL,
    },
    i2c, pac,
    prelude::*,
    pwm::{self, Channel},
    qei::{self, QeiOptions},
    serial,
    timer::{self, InputFilter, InputFpConfig, InternalFilterLength},
};

pub type Wheels = GenericWheels<
    pwm::Pwm<
        pac::TIM8,
        timer::Tim8NoRemap,
        (pwm::C2, pwm::C3),
        (
            Pin<Alternate<PushPull>, CRL, 'C', 7_u8>,
            Pin<Alternate<PushPull>, CRH, 'C', 8_u8>,
        ),
    >,
    qei::Qei<
        pac::TIM3,
        timer::Tim3NoRemap,
        (
            Pin<Input<Floating>, CRL, 'A', 6_u8>,
            Pin<Input<Floating>, CRL, 'A', 7_u8>,
        ),
    >,
    qei::Qei<
        pac::TIM4,
        timer::Tim4NoRemap,
        (
            Pin<Input<Floating>, CRL, 'B', 6_u8>,
            Pin<Input<Floating>, CRL, 'B', 7_u8>,
        ),
    >,
    ErasedPin<Output<PushPull>>,
>;

pub type Steering = GenericSteering<
    pwm::Pwm<pac::TIM1, timer::Tim1FullRemap, pwm::C1, Pin<Alternate<PushPull>, CRH, 'E', 9_u8>>,
>;

pub type DhTx = dma::TxDma<serial::Tx<pac::USART3>, dma::dma1::C2>;
pub type HdRx = dma::RxDma<serial::Rx<pac::USART3>, dma::dma1::C3>;

pub type Ahrs = mpu9250::Mpu9250<
    mpu9250::I2cDevice<
        i2c::BlockingI2c<
            pac::I2C1,
            (
                Pin<Alternate<OpenDrain>, CRH, 'B', 8_u8>,
                Pin<Alternate<OpenDrain>, CRH, 'B', 9_u8>,
            ),
        >,
    >,
    mpu9250::Marg,
>;

pub type Sr04EchoPin = Pin<Input<Floating>, CRL, 'D', 4_u8>;
pub type Sr04TriggerPin = Pin<Output<PushPull>, CRH, 'E', 13_u8>;

/// Starts up the board, returning resources provided by it.
pub fn startup(
    mut core: CorePeripherals,
    periph: pac::Peripherals,
) -> (
    Wheels,
    Steering,
    Analog,
    Sr04EchoPin,
    Sr04TriggerPin,
    DhTx,
    HdRx,
    Ahrs,
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
    let mut gpiod = periph.GPIOD.split();
    let mut gpioe = periph.GPIOE.split();

    // DMA1 initialization
    let dma1 = periph.DMA1.split();

    // Motor control initializationSr04TriggerPin
    let wheels = {
        let in1_l = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let in2_l = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

        let in1_r = gpioc.pc5.into_push_pull_output(&mut gpioc.crl);
        let in2_r = gpioe.pe12.into_push_pull_output(&mut gpioe.crh);

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
                    ifp_config: InputFpConfig {
                        ti1_filter: InputFilter::Internal(InternalFilterLength::Eight),
                        ti2_filter: InputFilter::Internal(InternalFilterLength::Eight),
                        ..Default::default()
                    },
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
                    ifp_config: InputFpConfig {
                        ti1_invert: true,
                        ti1_filter: InputFilter::Internal(InternalFilterLength::Eight),
                        ti2_filter: InputFilter::Internal(InternalFilterLength::Eight),
                        ..Default::default()
                    },
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
        let servo_ctrl = gpioe.pe9.into_alternate_push_pull(&mut gpioe.crh);
        let servo_ctrl_pwm =
            timer::Timer::tim1(periph.TIM1, &clocks).pwm(servo_ctrl, &mut afio.mapr, 50.hz());
        Steering::new(servo_ctrl_pwm, Channel::C1)
    };

    // Analog sensors initialization.
    let analog = {
        let vin = gpioc.pc4.into_analog(&mut gpioc.crl);

        Analog::new(vin, periph.ADC1, clocks)
    };

    // Ultrasonic sensor initialization.
    let echo = {
        let mut echo = gpiod.pd4.into_floating_input(&mut gpiod.crl);
        echo.make_interrupt_source(&mut afio);
        echo.enable_interrupt(&periph.EXTI);
        echo
    };

    let trig = gpioe
        .pe13
        .into_push_pull_output_with_state(&mut gpioe.crh, PinState::Low);

    // D -> H UART initialization.
    let (dh_tx, hd_rx) = {
        let tx = gpioc.pc10.into_alternate_push_pull(&mut gpioc.crh);
        let rx = gpioc.pc11.into_floating_input(&mut gpioc.crh);
        let mut dh_periph = serial::Serial::usart3(
            periph.USART3,
            (tx, rx),
            &mut afio.mapr,
            serial::Config {
                baudrate: 921600.bps(),
                parity: serial::Parity::ParityNone,
                stopbits: serial::StopBits::STOP1,
            },
            clocks,
        );
        dh_periph.listen(serial::Event::Idle);
        let (dh_tx_periph, hd_rx_periph) = dh_periph.split();

        let mut dh_tx = dh_tx_periph.with_dma(dma1.2);
        dh_tx.channel.listen(dma::Event::TransferComplete);

        let mut hd_rx = hd_rx_periph.with_dma(dma1.3);
        hd_rx.channel.listen(dma::Event::HalfTransfer);
        hd_rx.channel.listen(dma::Event::TransferComplete);

        (dh_tx, hd_rx)
    };

    // LrTimer initialization.
    //
    // Initialized before the MPU because we need to use it for delays.
    let mut lrtimer = LrTimer::new(periph.TIM2, &clocks);

    // Monotonic clock initialization.
    //
    // Initialized before the MPU because the I2C driver uses it for timeouts.
    let monotonic = clock::monotonic_setup(&mut core.DCB, core.DWT, core.SYST, clocks.sysclk().0);

    // MPU9250 initialization.
    // We're not using the interrupt for now - we poll it at 200Hz and the
    // sample rate should be high enough such that
    let mpu9250 = {
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let i2c = i2c::I2c::i2c1(
            periph.I2C1,
            (scl, sda),
            &mut afio.mapr,
            i2c::Mode::standard(400_000.hz()),
            clocks,
        )
        .blocking_default(clocks);
        let mut mpu9250 = mpu9250::Mpu9250::marg(
            i2c,
            &mut lrtimer,
            &mut mpu9250::MpuConfig::marg().mag_scale(mpu9250::MagScale::_16BITS),
        )
        .unwrap();
        match mpu9250.calibrate_at_rest::<_, [f32; 3]>(&mut lrtimer) {
            Err(e) => {
                defmt::error!(
                    "mpu9250: failed to perform bias calibration: {:?}",
                    defmt::Debug2Format(&e)
                );
                crate::exit();
            }
            Ok(acc_bias) => {
                defmt::info!("mpu9250: acc bias: {}", acc_bias);
            }
        }
        mpu9250
    };

    (
        wheels, steering, analog, echo, trig, dh_tx, hd_rx, mpu9250, lrtimer, monotonic,
    )
}
