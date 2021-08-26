use controller_core::board::lrtimer::Instant as LrInstant;
use controller_core::board::motion::{Angle, Duty};
use hdcomm_core::rpc::{MoveReqBody, PidParamUpdateReqBody, PidParams};
/// Board trajectory control.
use pid::Pid;
use rtic::rtic_monotonic::Milliseconds;
use smlang::statemachine;

use controller_fw::board::startup::{Steering, Wheels};

pub type Controller = StateMachine<ControllerContext>;

/// Context of a move request.
#[derive(PartialEq)]
pub struct MoveContext {
    /// Move request.
    pub request: MoveReqBody,
    /// Start time of move.
    start: LrInstant,
    /// Current s-curve time.
    s_curve_time: f32,
    /// End s-curve time.
    s_curve_time_end: f32,
}

impl MoveContext {
    pub fn new(req: MoveReqBody, time: LrInstant) -> Self {
        let duration = req.params.time_intervals.total_duration();
        Self {
            request: req,
            start: time,
            s_curve_time: 0.,
            s_curve_time_end: duration,
        }
    }
}

statemachine! {
    transitions: {
        *Idle + MoveRequest / start_steering_setup = SteeringSetup,
        Idle + PidParamUpdateReq(PidParamUpdateReqBody) / update_pid_parameters = Idle,
        SteeringSetup + SteeringSetupComplete / update_wheel_duty = MoveInProgress,
        SteeringSetup + MoveCancelRequest = CancelInProgress,
        MoveInProgress + MoveTimestep / update_wheel_duty = MoveInProgress,
        MoveInProgress + MoveComplete / stop_wheels = Idle,
        MoveInProgress + MoveCancelRequest = CancelInProgress,
        CancelInProgress + MoveCancelRequest = CancelInProgress,
        CancelInProgress + MoveCancelAcknowledgement / stop_wheels = Idle
    }
}

/// Context of the state machine.
pub struct ControllerContext {
    /// Left motor Pid controller state.
    pid_l: Pid<f32>,
    /// Right motor Pid controller state.
    pid_r: Pid<f32>,
    /// Position tracking PID loop update interval.
    pid_update_interval_ms: u32,
    /// PID-controlled wheels.
    wheels: Wheels,
    /// Front steering mechanism.
    steering: Steering,
    /// Move context.
    move_context: Option<MoveContext>,
    /// Initial wheel positions.
    initial_positions: Option<[i64; 2]>,
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

impl ControllerContext {
    /// Create a new state machine context.
    pub fn new(
        req: &PidParamUpdateReqBody,
        wheels: Wheels,
        steering: Steering,
    ) -> ControllerContext {
        Self {
            pid_l: pid_from_params(&req.params[0]),
            pid_r: pid_from_params(&req.params[1]),
            // FIXME: locked at 100 Hz for now!
            pid_update_interval_ms: 10,
            wheels,
            steering,
            move_context: None,
            initial_positions: None,
        }
    }
}

impl StateMachine<ControllerContext> {
    /// Polls the encoders to update their positions.
    pub fn poll_encoders(&mut self) {
        self.context_mut()
            .wheels
            .read_and_update_positions()
            .unwrap();
    }

    /// Submit a move request.
    ///
    /// Returns `true` if the move was accepted, `false` otherwise.
    pub fn move_request(&mut self, req: MoveReqBody, time: LrInstant) -> bool {
        if self.context().move_context.is_none() {
            self.context_mut().move_context = Some(MoveContext::new(req, time));
        } else {
            return false;
        }
        self.process_event(Events::MoveRequest).is_ok()
    }

    /// Submit a move cancellation request.
    ///
    /// Returns `true` if the cancel was queued, `false` otherwise.
    pub fn move_cancel(&mut self) -> bool {
        self.process_event(Events::MoveCancelRequest).is_ok()
    }

    /// Check if the move is complete.
    ///
    /// Returns `true` if the move is complete, `false` otherwise (e.g. not moving).
    fn move_complete(&self) -> bool {
        match self.state() {
            States::MoveInProgress => {
                self.context().move_context.as_ref().unwrap().s_curve_time
                    > self
                        .context()
                        .move_context
                        .as_ref()
                        .unwrap()
                        .s_curve_time_end
            }
            _ => false,
        }
    }

    /// Called when the periodic task is servicing the controller.
    ///
    /// Returns the next event to pass in and when to execute it.
    ///
    /// If `None`, the periodic task is to be stopped.
    pub fn service(&mut self, ev: Events) -> Option<(Events, Milliseconds)> {
        if let States::CancelInProgress = self.state() {
            self.process_event(Events::MoveCancelAcknowledgement)
                .unwrap();
            return None;
        }

        match self.process_event(ev).unwrap() {
            States::MoveInProgress => {
                if self.move_complete() {
                    defmt::info!("queued move complete!");
                    Some((Events::MoveComplete, Milliseconds(0_u32)))
                } else {
                    Some((
                        Events::MoveTimestep,
                        Milliseconds(self.context().pid_update_interval_ms),
                    ))
                }
            }
            States::SteeringSetup => Some((
                Events::SteeringSetupComplete,
                Milliseconds(self.context().pid_update_interval_ms),
            )),
            _ => None,
        }
    }
}

impl StateMachineContext for ControllerContext {
    fn update_pid_parameters(&mut self, req: &PidParamUpdateReqBody) {
        self.pid_l = pid_from_params(&req.params[0]);
        self.pid_r = pid_from_params(&req.params[1]);
        self.pid_update_interval_ms = req.update_interval_ms.into();
    }

    fn start_steering_setup(&mut self) {
        let mctx = self.move_context.as_ref().unwrap();

        // TODO: May need to reset previous measurement too: only possible with
        // new PID structure.
        self.pid_l.reset_integral_term();
        self.pid_r.reset_integral_term();
        self.steering.set(Angle::from_num(mctx.request.steering));
        self.stop_wheels();

        self.initial_positions = Some(self.wheels.read_and_update_positions().unwrap());
        if crate::app::trajectory_controller_service::spawn_after(
            Milliseconds(
                self.move_context
                    .as_ref()
                    .unwrap()
                    .request
                    .steering_setup_ms as u32,
            ),
            Events::SteeringSetupComplete,
        )
        .is_err()
        {
            panic!("trajectory controller service no buffer")
        };
    }

    fn update_wheel_duty(&mut self) {
        let mctx = self.move_context.as_mut().unwrap();

        let elapsed = mctx.s_curve_time;

        let initial = self.initial_positions.unwrap();
        let target_fast = s_curve::eval_position(&mctx.request.params, elapsed);
        let target_slow = target_fast * mctx.request.ratio;

        let (duty_l, duty_r) = {
            if mctx.request.ref_left {
                self.pid_l.setpoint = target_fast + initial[0] as f32;
                self.pid_r.setpoint = target_slow + initial[1] as f32;
            } else {
                self.pid_l.setpoint = target_slow + initial[0] as f32;
                self.pid_r.setpoint = target_fast + initial[1] as f32;
            }

            let positions = self.wheels.read_and_update_positions().unwrap();
            (
                self.pid_l.next_control_output(positions[0] as f32).output,
                self.pid_r.next_control_output(positions[1] as f32).output,
            )
        };

        self.wheels.drive(
            controller_core::board::motion::Wheel::LEFT,
            Duty::from_num(duty_l),
        );
        self.wheels.drive(
            controller_core::board::motion::Wheel::RIGHT,
            Duty::from_num(duty_r),
        );

        mctx.s_curve_time += self.pid_update_interval_ms as f32 * 1e-3;
    }

    fn stop_wheels(&mut self) {
        self.move_context = None;
        self.wheels.drive(
            controller_core::board::motion::Wheel::LEFT,
            Duty::from_num(0),
        );
        self.wheels.drive(
            controller_core::board::motion::Wheel::RIGHT,
            Duty::from_num(0),
        );
    }
}
