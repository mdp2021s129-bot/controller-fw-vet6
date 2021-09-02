use controller_core::board::lrtimer::Instant as LrInstant;
use controller_fw::board::clock::RTICMonotonic;
use core::convert::TryInto;
use embedded_hal::digital::v2::StatefulOutputPin;
use embedded_time::{duration::Microseconds, Instant};
use fixed::types::U16F16 as DistanceImpl;
use fixed_macro::types::U16F16 as distance;
/// Driver for the HC-SR04 ultrasonic sensor.

/// Distance type used for measurements.
///
/// In unis of meters.
pub type Distance = DistanceImpl;

/// Time after the start of the measurement beyond which we consider the
/// measurement to have timed out.
///
/// The HC-SR04 datasheet suggests a timeout of 60 milliseconds.
pub const TIMEOUT: Microseconds = Microseconds(60_000);

/// Minimum width of the trigger pulse.
pub const TRIGGER_WIDTH: Microseconds = Microseconds(10);

/// Convers the width of the ECHO pulse (in units of microseconds)
/// to meters.
///
/// Based on the speed of sound at 25 deg C (346.06 m/s).
pub const SCALING_FACTOR: Distance = distance!(0.00017303);

/// Minimum width of the echo pulse.
///
/// Any pulses with widths smaller than this are considered to be glitches.
pub const MINIMUM_ECHO_WIDTH: Microseconds = Microseconds(200);

enum MeasurementState {
    /// Trigger pin has been pulled high.
    AfterTriggerRising,
    /// Trigger pin has been pulled low after it has been high for
    /// `TRIGGER_WIDTH`.
    AfterTriggerFalling,
    /// After a rising edge has been detected on the echo pin.
    AfterEchoRising {
        /// Time at which the rising edge was detected on the pin.
        rise: Instant<RTICMonotonic>,
    },
}

/// Sensor state.
enum State {
    /// Sensor is idle - no measurement has been started.
    Idle,
    /// A measurement has been started.
    Measuring {
        /// Measurement start time.
        start: LrInstant,
        /// Measurement state.
        state: MeasurementState,
    },
}

/// Measurement information.
#[derive(Debug, Clone)]
pub struct Measurement {
    /// Measurement start time.
    pub start: LrInstant,
    /// Measurement end time.
    pub end: LrInstant,
    /// Measurement result.
    pub result: Result<Distance, Error>,
}

/// Errors that can be returned from the sensor.
#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// A measurement is already in progress.
    InProgress,
    /// Measurement timed out.
    Timeout,
    /// Sensor measured a distance that was abnormally short.
    TooShort,
    /// An unexpected event was provided.
    Unexpected,
}

/// Events that can be passed to the driver.
pub enum Event {
    /// A duration of at `TRIGGER_WIDTH` has passed since `trigger()` returned
    /// `Ok()`.
    TriggerComplete,
    /// An rising / falling edge interrupt occurred on the echo pin.
    EchoInterrupt,
}

/// Driver structure.
///
/// `TRIG`: Trigger pin.LRCLOCK
/// `HRCLOCK`: High-resolution (microsecond-level) clock.
/// `LRCLOCK`: Low-resolution clock used to record the start and end timestamps,
///            as well as detect timeouts.
pub struct Sr04<TRIG> {
    /// Trigger pin.
    trig: TRIG,
    /// State of the driver.
    state: State,
    /// Last measurement recorded.
    last: Option<Measurement>,
}

impl<TRIG: StatefulOutputPin> Sr04<TRIG> {
    /// Create a new `Sr04` instance.
    pub fn new(trig: TRIG) -> Self {
        Self {
            trig,
            state: State::Idle,
            last: None,
        }
    }

    /// Trigger the sensor.
    ///
    /// An `Ok()` result requires that the caller pass `Event::TriggerComplete`
    /// to `process()` after a duration of `TRIGGER_WIDTH`.
    pub fn trigger(&mut self, at: LrInstant) -> Result<(), Error> {
        self.poll(at);
        match self.state {
            State::Idle => {
                self.trig.set_high().ok();

                self.state = State::Measuring {
                    start: at,
                    state: MeasurementState::AfterTriggerRising,
                };

                Ok(())
            }
            _ => Err(Error::InProgress),
        }
    }

    /// Obtain the last complete measurement, if any.
    pub fn measurement(&mut self, at: LrInstant) -> Option<&Measurement> {
        self.poll(at);

        self.last.as_ref()
    }

    /// Handles time-based driver state machine transitions.
    ///
    /// Returns `true` if this resulted in a measurement being completed.
    fn poll(&mut self, at: LrInstant) -> bool {
        match self.state {
            State::Measuring { start, .. } => {
                let elapsed: Microseconds<u32> = (at - start).try_into().unwrap_or(TIMEOUT);
                if elapsed >= TIMEOUT {
                    self.state = State::Idle;
                    self.last = Some(Measurement {
                        start,
                        end: at,
                        result: Err(Error::Timeout),
                    });
                    true
                } else {
                    false
                }
            }
            _ => false,
        }
    }

    /// Process a event.
    ///
    /// Returns `Ok(true)` if the event resulted in a measurement being
    /// completed.
    pub fn callback(&mut self, event: Event, at: LrInstant) -> Result<bool, Error> {
        if self.poll(at) {
            return Ok(true);
        }

        match self.state {
            State::Idle => Err(Error::Unexpected),
            State::Measuring {
                start,
                ref mut state,
            } => {
                match state {
                    MeasurementState::AfterTriggerRising => {
                        if let Event::TriggerComplete = event {
                            self.trig.set_low().ok();
                            *state = MeasurementState::AfterTriggerFalling;
                        } else {
                            return Err(Error::Unexpected);
                        }
                    }
                    MeasurementState::AfterTriggerFalling => {
                        if let Event::EchoInterrupt = event {
                            *state = MeasurementState::AfterEchoRising {
                                rise: crate::app::monotonics::Dwt::now(),
                            };
                        } else {
                            return Err(Error::Unexpected);
                        }
                    }
                    MeasurementState::AfterEchoRising { rise } => {
                        if let Event::EchoInterrupt = event {
                            // Clamp width to timeout.
                            // Because it should be impossible for the width to exceed 60_000us
                            // unless the two timers are derived from the same clock / have
                            // significantly different precision.
                            let fall = crate::app::monotonics::Dwt::now();
                            let echo_duration: Microseconds<u32> = core::cmp::min(
                                (fall - *rise).try_into().unwrap_or(TIMEOUT),
                                TIMEOUT,
                            );
                            self.last = Some(Measurement {
                                start,
                                end: at,
                                result: if echo_duration < MINIMUM_ECHO_WIDTH {
                                    Err(Error::TooShort)
                                } else {
                                    // echo_duration.0 guaranteed to be smaller than max(u16) because
                                    // of clamp.
                                    Ok(Distance::from_num(echo_duration.0 as u16) * SCALING_FACTOR)
                                },
                            });
                            self.state = State::Idle;
                            return Ok(true);
                        } else {
                            return Err(Error::Unexpected);
                        }
                    }
                }
                Ok(false)
            }
        }
    }
}
