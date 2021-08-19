use cortex_m::peripheral::{DCB, DWT, SYST};
use dwt_systick_monotonic::DwtSystick;
/// Controller clocks.
use stm32f1xx_hal::{flash::ACR, prelude::*, rcc::Rcc};

/// Core clock frequency.
pub const SYSCLK_HZ: u32 = 72_000_000;

/// Type of the RTIC Monotonic clock.
pub type RTICMonotonic = DwtSystick<SYSCLK_HZ>;

/// Sets up the microcontroller controller's clock tree.
///
/// Configuration:
///
/// - 8MHz HSE.
/// - 72MHz core.
/// - 32MHz APB1.
/// - 72MHz APB2.
pub fn clock_tree_setup(mut acr: ACR, rcc: Rcc) -> stm32f1xx_hal::rcc::Clocks {
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .sysclk(72.mhz())
        .adcclk(12.mhz())
        .freeze(&mut acr);

    defmt::info!(
        "clock freqs: hclk: {}, pclk1: {}, pclk2: {}, sysclk: {}, adcclk: {}",
        clocks.hclk().0,
        clocks.pclk1().0,
        clocks.pclk2().0,
        clocks.sysclk().0,
        clocks.adcclk().0
    );

    clocks
}

/// Sets up the monotonic clock.
pub fn monotonic_setup(dcb: &mut DCB, dwt: DWT, syst: SYST, sysclk: u32) -> RTICMonotonic {
    DwtSystick::new(dcb, dwt, syst, sysclk)
}
