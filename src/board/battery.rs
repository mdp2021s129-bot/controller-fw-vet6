use fixed::FixedU32;
use nb::block;
/// Battery monitoring abstraction for the controller.
///
/// The controller provides VIN/11 on PC4.
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpioc, Analog},
    pac,
    prelude::*,
};

/// VIN voltage divider divide ratio.
const DIVIDE_RATIO: u16 = 11;

type Voltage = FixedU32<fixed::types::extra::U16>;

// TODO: if we have more analog sensors, take control of the ADC
// so we don't impose these insane requirements on users.
// Have some sort of Analog structure that encapsulates all
// analog sensors read by the controller through its ADC.

pub struct Battery {
    /// Take pin here to force PC4 to analog mode.
    pin: gpioc::PC4<Analog>,
}

impl Battery {
    /// Create a new `Battery` instance.
    fn new(pin: gpioc::PC4<Analog>) -> Self {
        Self { pin: pin }
    }

    /// Read the battery voltage from ADC1.
    /// The ADC's result must be right aligned.
    ///
    /// Returns the battery's voltage level in Volts.
    fn read(&mut self, adc: &mut Adc<pac::ADC1>) -> Voltage {
        let word: u16 = block!(adc.read(&mut self.pin)).unwrap();
        Voltage::from(word) * Voltage::from(DIVIDE_RATIO)
    }
}
