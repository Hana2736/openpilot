import math
import numpy as np

from cereal import log
from openpilot.selfdrive.car.interfaces import FRICTION_THRESHOLD
from openpilot.selfdrive.controls.lib.drive_helpers import get_friction
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    # PID runs in torque space (see update()), so its limits are the raw torque limits.
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max, rate=1/self.dt)
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, llk, model_data, frogpilot_toggles):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      actual_lateral_accel = measured_curvature * CS.vEgo ** 2

      low_speed_factor = np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) ** 2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * measured_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation

      # Error in torque space: pass setpoint and measurement through the siglin separately, then
      # subtract. In the saturated region siglin(setpoint) ~= siglin(measurement) ~= +/-0.5*b, so the
      # error collapses toward zero and the controller backs off instead of railing -> anti-ping-pong.
      # (The older lat-accel-space form kept a large error in saturation and re-saturated the PID
      #  output through the siglin, which turns an over-stiff tune into oscillation.)
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params)
      pid_log.error = float(torque_from_setpoint - torque_from_measurement)

      # Feedforward in torque space: roll/offset-corrected desired lat accel through the siglin (the
      # siglin's 'd' offset is preserved here), plus friction (get_friction is already torque-space).
      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel - self.torque_params.latAccelOffset, self.torque_params)
      ff += get_friction(desired_lateral_accel - actual_lateral_accel, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      self.pid._k_p = frogpilot_toggles.steerKp
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
