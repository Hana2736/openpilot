#!/usr/bin/env python3
from math import exp, fabs
import numpy as np

from cereal import car, custom
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.mazda.values import CAR, LKAS_LIMITS, MazdaFlags, GEN1, GEN2, GEN2_LATERAL_TUNING
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

# Set to True to use speed-varying sigmoid+linear torque model
USE_SPEED_VARYING_LATERAL = True


class CarInterface(CarInterfaceBase):
  # Mutable container for current speed - updated in _update(), read by torque callback
  _v_ego = [0.0]

  def get_abc_for_speed(self, v_ego: float) -> tuple[float, float, float]:
    """Get speed-varying A, B, C params for sigmoid+linear torque model."""
    p = GEN2_LATERAL_TUNING.get(MazdaFlags.GEN2)
    if p is None:
      return 5.0, 0.8, 0.15  # defaults
    a = float(np.interp(v_ego, p.speed_bp, p.a_vals))
    b = float(np.interp(v_ego, p.speed_bp, p.b_vals))
    c = float(np.interp(v_ego, p.speed_bp, p.c_vals))
    return a, b, c

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if USE_SPEED_VARYING_LATERAL and self.CP.carFingerprint in GEN2:
      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: car.CarParams.LateralTorqueTuning):
        v_ego = CarInterface._v_ego[0]
        a, b, c = self.get_abc_for_speed(v_ego)
        sig_input = a * lateral_acceleration
        sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
        steer_torque = (sig * b) + (lateral_acceleration * c)
        return float(steer_torque)
      return torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    if USE_SPEED_VARYING_LATERAL and self.CP.carFingerprint in GEN2:
      def lateral_accel_from_torque_siglin(torque: float, torque_params: car.CarParams.LateralTorqueTuning):
        v_ego = CarInterface._v_ego[0]
        a, b, c = self.get_abc_for_speed(v_ego)
        # Numerical inverse - build lookup for current speed
        lataccel_values = np.arange(-8.0, 8.0, 0.01)
        sig_input = a * lataccel_values
        sig = np.sign(sig_input) * (1 / (1 + np.exp(-np.abs(sig_input))) - 0.5)
        torque_values = (sig * b) + (lataccel_values * c)
        return float(np.interp(torque, torque_values, lataccel_values))
      return lateral_accel_from_torque_siglin
    else:
      return self.lateral_accel_from_torque_linear

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    ret.carName = "mazda"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True
    ret.dashcamOnly = False
    ret.openpilotLongitudinalControl = True
    p = Params()
    if p.get_bool("ManualTransmission"):
      ret.flags |= MazdaFlags.MANUAL_TRANSMISSION.value
      ret.transmissionType = car.CarParams.TransmissionType.manual
    else:
      ret.transmissionType = car.CarParams.TransmissionType.automatic

    if candidate in GEN1:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_GEN1
      if p.get_bool("TorqueInterceptorEnabled"): # Torque Interceptor Installed
        print("Torque Interceptor Installed")
        ret.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_TORQUE_INTERCEPTOR
      if p.get_bool("RadarInterceptorEnabled"): # Radar Interceptor Installed
        ret.flags |= MazdaFlags.RADAR_INTERCEPTOR.value
        ret.experimentalLongitudinalAvailable = True
        ret.radarUnavailable = False
        ret.startingState = True
        ret.longitudinalTuning.kpBP = [0., 5., 30.]
        ret.longitudinalTuning.kpV = [1.3, 1.0, 0.7]
        ret.longitudinalTuning.kiBP = [0., 5., 20., 30.]
        ret.longitudinalTuning.kiV = [0.36, 0.23, 0.17, 0.1]
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_RADAR_INTERCEPTOR

      if p.get_bool("NoMRCC"): # No Mazda Radar Cruise Control; Missing CRZ_CTRL signal
        ret.flags |= MazdaFlags.NO_MRCC.value
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_NO_MRCC
      if p.get_bool("NoFSC"):  # No Front Sensing Camera
        ret.flags |= MazdaFlags.NO_FSC.value
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_NO_FSC

      ret.steerActuatorDelay = 0.1
      ret.enableBsm = True

    if candidate in GEN2:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_MAZDA_GEN2
      ret.experimentalLongitudinalAvailable = True
      ret.stopAccel = -.5
      ret.vEgoStarting = .2
      ret.longitudinalActuatorDelay = 0.35 # gas is 0.25s and brake looks like 0.5
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [0.0, 0.0, 0.0]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.1, 0.1]
      ret.startingState = True
      ret.steerActuatorDelay = 0.335

    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_3_2019, CAR.MAZDA_CX_30, CAR.MAZDA_CX_50) and not ret.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  # returns a car.CarState
  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, self.cp_body, frogpilot_toggles)

    # Update speed for lateral torque callback
    CarInterface._v_ego[0] = ret.vEgo

     # TODO: add button types for inc and dec
    ret.buttonEvents = [
      *create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas}),
    ]

    # events
    events = self.create_common_events(ret)

    if self.CP.flags & MazdaFlags.GEN1:
      if self.CS.lkas_disabled:
        events.add(EventName.lkasDisabled)
      elif self.CS.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

      if not self.CS.acc_active_last and not self.CS.ti_lkas_allowed:
        events.add(EventName.steerTempUnavailable)
      #if (not self.CS.ti_lkas_allowed) and (self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR):
      #  events.add(EventName.steerTempUnavailable) # torqueInterceptorTemporaryWarning

    ret.events = events.to_msg()

    return ret, fp_ret
