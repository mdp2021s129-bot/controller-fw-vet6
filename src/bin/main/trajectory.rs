use controller_core::board::motion::{Angle, Duty};
use controller_core::board::{lrtimer::Instant as LrInstant, motion::Wheel};
use controller_fw::board::startup::{Steering, Wheels};
use debugless_unwrap::DebuglessUnwrap;
use hdcomm_core::rpc::{MoveReqBody, PidParamUpdateReqBody, PidParams, RawTeleOpReqBody};
/// Board trajectory control.
use pid::Pid;
use rtic::rtic_monotonic::Milliseconds;

/// Controller state.
enum State {
    /// Controller is Idle.
    ///
    /// No move has been requested from the controller.
    Idle,
    /// Controller is active.
    ///
    /// A move has been requested from the controller and the controlelr is
    /// executing it.
    Active(MoveContext),
}

/// Context of a move request.
struct MoveContext {
    /// State of the move.
    state: MoveState,

    /// Move request.
    request: MoveReqBody,
    /// Start time of move.
    start: LrInstant,
    /// Handle to any created callback task.
    spawn_handle: Option<crate::app::trajectory_controller_callback::SpawnHandle>,
}

impl MoveContext {
    /// Creates new move context.
    ///
    /// The move state is initialized to the `SteeringReset` state.
    fn new(req: MoveReqBody, time: LrInstant) -> Self {
        Self {
            request: req,
            start: time,
            spawn_handle: None,
            state: MoveState::SteeringReset,
        }
    }
}

/// State of a move.
#[derive(Debug, Clone, PartialEq)]
enum MoveState {
    /// The controller is resetting the steering.
    ///
    /// It exists in this state for 1/2 the steering setup time.
    SteeringReset,
    /// The controller is setting up the steering.
    ///
    /// It exists in this state for 1/2 the steering setup time.
    SteeringSetup,
    /// The controller is actively driving the motors.
    ///
    /// It exists in this state for as long as the planned S-curve.
    Driving(DriveContext),
}

/// Context held in the drive state.
#[derive(Debug, Clone, PartialEq)]
struct DriveContext {
    /// Current S-curve time point.
    current_time: f32,
    /// Total S-curve duration.
    end_time: f32,

    /// Left & right position PID controllers.
    controllers: [Pid<f32>; 2],
    /// Position PID controller update rate.
    update_interval_ms: u32,
    /// Left & right encoder offsets at the start of the move (because the
    /// S-curve generated position targets are zero-referenced.
    offsets: [i64; 2],
}

impl DriveContext {
    /// Update the PID controllers with information about the robot's state,
    /// allowing it to generate duty cycles for the current time step.
    ///
    /// Returns `None` if the driving state should be ended.
    fn update(&mut self, request: &MoveReqBody, mut positions: [i64; 2]) -> Option<[Duty; 2]> {
        let elapsed = self.current_time;
        if elapsed >= self.end_time {
            return None;
        }

        self.current_time += self.update_interval_ms as f32 * 1e-3;
        let target_ref = s_curve::eval_position(&request.params, elapsed)
            * if request.reverse { -1. } else { 1. };

        let target_follower = target_ref * request.ratio;
        let mut targets = [target_ref, target_follower];

        // Swap the reference over to the other motor if the left motor isn't the reference.
        if !request.ref_left {
            targets.swap(0, 1);
        }

        // Compensate for position offsets so that the control loop can be zero-referenced.
        positions[0] -= self.offsets[0];
        positions[1] -= self.offsets[1];

        self.controllers[0].setpoint = targets[0];
        self.controllers[1].setpoint = targets[1];

        let errors = [
            targets[0] - positions[0] as f32,
            targets[1] - positions[1] as f32,
        ];
        let duties = [
            self.controllers[0]
                .next_control_output(positions[0] as f32)
                .output,
            self.controllers[1]
                .next_control_output(positions[1] as f32)
                .output,
        ];
        defmt::debug!(
            "PID, {},{},{},{},{},{}",
            errors[0],
            duties[0],
            errors[1],
            duties[1],
            positions[0],
            positions[1]
        );
        Some([Duty::from_num(duties[0]), Duty::from_num(duties[1])])
    }
}

/// Robot trajectory controller.
///
/// Arbitrates access to the robot's drive wheels & steering.
pub struct Controller {
    /// Controller state.
    state: State,

    /// Parameters for the position tracking PID controllers.
    pid_parameters: [PidParams; 2],
    /// Position tracking PID controller update interval.
    pid_update_interval_ms: u32,

    /// Drive wheels.
    wheels: Wheels,
    /// Steering servo.
    steering: Steering,
}

/// Callback types.
pub enum CallbackKind {
    /// Callback is meant to poll encoders.
    EncoderPoll,
    /// Callback is meant to drive the state machine.
    Statemachine,
}

/// Controller status.
pub enum Status {
    /// Controller is idle.
    Idle,
    /// Controller is actively performing a move.
    Active {
        /// Move start time.
        start: LrInstant,
        /// Time required for move to complete (in seconds).
        required: f32,
    },
}

/// Create a Pid instance from a set of Pid parameters.
///
/// The default setpoint is zero.
fn pid_from_params(params: &PidParams) -> Pid<f32> {
    Pid::new(
        params.kp,
        params.ki,
        params.kd,
        params.p_limit,
        params.i_limit,
        params.d_limit,
        params.output_limit,
        0.0,
    )
}

impl Controller {
    /// Create a new controller.
    ///
    /// Also starts the controller's periodic callbacks.
    pub fn new(req: &PidParamUpdateReqBody, wheels: Wheels, steering: Steering) -> Self {
        crate::app::trajectory_controller_callback::spawn_after(
            Milliseconds(1000_u32),
            CallbackKind::EncoderPoll,
        )
        .ok();

        Self {
            pid_parameters: req.params.clone(),
            pid_update_interval_ms: req.update_interval_ms as u32,
            wheels,
            state: State::Idle,
            steering,
        }
    }

    /// Submits a move request.
    ///
    /// Returns `true` if the move was accepted, `false` otherwise.
    #[warn(unused_results)]
    pub fn request_move(&mut self, req: MoveReqBody, time: LrInstant) -> bool {
        if !matches!(self.state, State::Idle) {
            return false;
        }

        let handle = crate::app::trajectory_controller_callback::spawn_after(
            Milliseconds((req.steering_setup_ms as u32) / 2),
            CallbackKind::Statemachine,
        )
        .debugless_unwrap();

        // Stop wheels just in case a teleop was being performed.
        self.stop_wheels();
        self.steering.set(Angle::from_num(
            (((req.steering + 0.46) + 0.35) % 0.7) - 0.46,
        ));
        let mut move_ctx = MoveContext::new(req, time);
        move_ctx.spawn_handle = Some(handle);
        let new_state = State::Active(move_ctx);
        self.state = new_state;

        true
    }

    /// Cancel an active move.
    ///
    /// Returns `true` if a move was cancelled, `false` if no move was
    /// in progress.
    pub fn cancel_move(&mut self) -> bool {
        if let State::Active(ref mut move_context) = self.state {
            if let Some(handle) = move_context.spawn_handle.take() {
                // It doesn't matter if it expired.
                handle.cancel().ok();
            }

            self.stop_wheels();
            self.state = State::Idle;
            true
        } else {
            false
        }
    }

    /// Change the PID paramters used.
    ///
    /// Can only be called if a move is not in progress.
    ///
    /// Returns `true` if the change was successful, `false` if not.
    pub fn set_pid_parameters(&mut self, req: &PidParamUpdateReqBody) -> bool {
        if let State::Idle = self.state {
            self.pid_parameters = req.params.clone();
            self.pid_update_interval_ms = req.update_interval_ms as u32;

            true
        } else {
            false
        }
    }

    /// Checks the controller's status.
    pub fn status(&self) -> Status {
        match &self.state {
            State::Idle => Status::Idle,
            State::Active(move_context) => Status::Active {
                required: move_context.request.time_required(),
                start: move_context.start,
            },
        }
    }

    /// Polls the encoders to update their positions.
    pub fn poll_encoders(&mut self) -> [i64; 2] {
        self.wheels.read_and_update_positions().unwrap()
    }

    /// Sends raw commands to the steering servo & drive wheels.
    ///
    /// Returns `true` if the command was successful, `false` if not.
    pub fn raw_control(&mut self, req: &RawTeleOpReqBody) -> bool {
        if let State::Idle = self.state {
            if let Some(position) = req.steering {
                self.steering.set(Angle::from_num(position))
            }

            if let Some(duty) = req.wheel_l {
                self.wheels.drive(Wheel::LEFT, Duty::from_num(duty));
            }

            if let Some(duty) = req.wheel_r {
                self.wheels.drive(Wheel::RIGHT, Duty::from_num(duty));
            }

            true
        } else {
            false
        }
    }

    /// Stops both wheels by putting them in brake mode.
    fn stop_wheels(&mut self) {
        self.wheels.drive(Wheel::LEFT, Duty::from_num(0));
        self.wheels.drive(Wheel::RIGHT, Duty::from_num(0));
    }

    /// Handles callbacks from the delayed-work task.
    pub fn callback(&mut self, kind: CallbackKind) {
        let now = crate::app::monotonics::Dwt::now();
        use CallbackKind::*;

        match kind {
            EncoderPoll => {
                self.poll_encoders();
                crate::app::trajectory_controller_callback::spawn_after(
                    Milliseconds(1000_u32),
                    CallbackKind::EncoderPoll,
                )
                .debugless_unwrap();
            }
            Statemachine => {
                match &mut self.state {
                    State::Idle => panic!("Statemachine callback received when idle"),
                    State::Active(ref mut move_context) => {
                        match &mut move_context.state {
                            MoveState::SteeringReset => {
                                self.steering
                                    .set(Angle::from_num(move_context.request.steering));
                                move_context.state = MoveState::SteeringSetup;
                                move_context.spawn_handle = Some(
                                    crate::app::trajectory_controller_callback::spawn_after(
                                        Milliseconds(
                                            (move_context.request.steering_setup_ms as u32) / 2,
                                        ),
                                        CallbackKind::Statemachine,
                                    )
                                    .debugless_unwrap(),
                                );
                            }
                            MoveState::SteeringSetup => {
                                // We need to transition from the steering setup state to the
                                // driving state.
                                let drive_context = {
                                    let offsets = self.wheels.read_and_update_positions().unwrap();
                                    let current_time = 0_f32;
                                    let controllers = [
                                        pid_from_params(&self.pid_parameters[0]),
                                        pid_from_params(&self.pid_parameters[1]),
                                    ];

                                    let end_time =
                                        move_context.request.params.time_intervals.total_duration();
                                    let update_interval_ms = self.pid_update_interval_ms;

                                    DriveContext {
                                        offsets,
                                        current_time,
                                        controllers,
                                        end_time,
                                        update_interval_ms,
                                    }
                                };
                                move_context.state = MoveState::Driving(drive_context);
                                move_context.spawn_handle = Some(
                                    crate::app::trajectory_controller_callback::spawn_after(
                                        Milliseconds(0_u32),
                                        CallbackKind::Statemachine,
                                    )
                                    .debugless_unwrap(),
                                );
                            }
                            MoveState::Driving(ref mut drive_context) => {
                                // Calculate the duty cycle for the current iteration & apply them.
                                let positions = self.wheels.read_and_update_positions().unwrap();
                                let update_ms = drive_context.update_interval_ms;
                                match drive_context.update(&move_context.request, positions) {
                                    Some(duties) => {
                                        self.wheels.drive(Wheel::LEFT, duties[0]);
                                        self.wheels.drive(Wheel::RIGHT, duties[1]);

                                        // We want to be accurate and not let the variable update time
                                        // skew our update frequency.
                                        move_context.spawn_handle = Some(
                                            crate::app::trajectory_controller_callback::spawn_at(
                                                now + Milliseconds(update_ms),
                                                CallbackKind::Statemachine,
                                            )
                                            .debugless_unwrap(),
                                        );
                                    }
                                    None => {
                                        // End move.
                                        self.stop_wheels();
                                        self.state = State::Idle;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
