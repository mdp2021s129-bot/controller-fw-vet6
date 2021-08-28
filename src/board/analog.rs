use fixed::FixedU32;
use nb::block;
/// Analog sensor abstractions for the controller.
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpioc, Analog as AnalogMode},
    pac,
    prelude::*,
    rcc::Clocks,
};

/// The controller provides VIN/11 on PC4.
///
/// This constant provides the VIN voltage divider divide ratio.
pub const VIN_DIVIDE_RATIO: u16 = 11;

/// Voltage type.
///
/// In units of volts.
pub type Voltage = FixedU32<fixed::types::extra::U16>;

/// Number of least significant bits per volt.
pub const LSBS_PER_VOLT: Voltage = Voltage::from_be_bytes([0x04, 0xd9, 0x36, 0x4e]);

/// Abstraction over all analog sensors of the robot.
///
/// Provides only the VIN reading for now.
pub struct Analog {
    vin: gpioc::PC4<AnalogMode>,
    adc: Adc<pac::ADC1>,
}

impl Analog {
    /// Create a new `Analog` instance.
    pub fn new(vin: gpioc::PC4<AnalogMode>, adc: pac::ADC1, clocks: Clocks) -> Self {
        let mut adc = Adc::adc1(adc, clocks);
        adc.set_align(stm32f1xx_hal::adc::Align::Right);
        adc.set_sample_time(stm32f1xx_hal::adc::SampleTime::T_1);

        Self { vin, adc }
    }

    /// Read the voltage level of the VIN bus.
    ///
    /// Blocking.
    pub fn vin(&mut self) -> Voltage {
        let word: u16 = block!(self.adc.read(&mut self.vin)).unwrap();
        (Voltage::from(word) / LSBS_PER_VOLT) * Voltage::from(VIN_DIVIDE_RATIO)
    }
}
