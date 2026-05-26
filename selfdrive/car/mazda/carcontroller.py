import numpy as np
from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, MazdaFlags
from openpilot.common.realtime import ControlsTimer as Timer, DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

import cereal.messaging as messaging

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# Safety clip for the static-map inverse path.  The stock affine
# (`a*200 + 2000`) reaches roughly [1400, 2400] in worst-case brake/accel;
# we give the inverse-table output a slightly wider envelope so a small
# fit error never silently saturates, but cap absolute extremes since
# values far outside this range would be Mazda's onboard ACC ignoring or
# misinterpreting the command.  Stock affine output is intentionally NOT
# clipped here - that path keeps its pre-existing behavior.
_GEN2_CMD_LO = 1200
_GEN2_CMD_HI = 2600


def _compute_can_cmd(target_accel, v_ego, p, frogpilot_toggles):
  """Map target_accel (m/s²) -> 12-bit ACCEL_CMD CAN integer.

  Default: the stock global affine `target_accel * accel_scale + accel_offset`,
  unclipped (preserves existing behavior so toggling the static map off
  reverts exactly).

  Table path: per-bin pwl-deadband inverse interpolated at v_ego.
    target_accel > 0:  can = target_accel / s_pos + dz_hi
    target_accel < 0:  can = target_accel / s_neg + dz_lo
    target_accel == 0: can = midpoint of [dz_lo, dz_hi]
  Output hard-clipped to [_GEN2_CMD_LO, _GEN2_CMD_HI].
  """
  if not getattr(frogpilot_toggles, "use_long_static_map", False):
    return int(target_accel * p.accel_scale + p.accel_offset)

  tbl = frogpilot_toggles.long_static_map_table  # tuple of (v, dz_lo, dz_hi, s_pos, s_neg)
  vs = [row[0] for row in tbl]
  dz_lo = float(np.interp(v_ego, vs, [row[1] for row in tbl]))
  dz_hi = float(np.interp(v_ego, vs, [row[2] for row in tbl]))
  s_pos = float(np.interp(v_ego, vs, [row[3] for row in tbl]))
  s_neg = float(np.interp(v_ego, vs, [row[4] for row in tbl]))

  if target_accel > 0:
    can = target_accel / max(s_pos, 1e-6) + dz_hi
  elif target_accel < 0:
    can = target_accel / max(s_neg, 1e-6) + dz_lo
  else:
    can = 0.5 * (dz_lo + dz_hi)
  return int(np.clip(can, _GEN2_CMD_LO, _GEN2_CMD_HI))


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.ti_apply_steer_last = 0
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0
    self.ccp = CarControllerParams(CP)
    self.hold_timer = Timer(6.0)
    self.hold_delay = Timer(.5) # delay before we start holding as to not hit the brakes too hard
    self.resume_timer = Timer(0.5)
    self.cancel_delay = Timer(0.07) # 70ms delay to try to avoid a race condition with stock system
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.blend_coeff = 0 #factor for blending OP and stock long. 0 is fully stock, 1 is fully OP
    self.transition_time = 2.5 #After this number of seconds, the smooth blending from stock to OP (or vice versa) is complete
    self.distance_last = None
    self.sm = messaging.SubMaster(['longitudinalPlan', 'radarState'])


  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    self.sm.update(0)
    long_plan = self.sm['longitudinalPlan']
    allow_throttle = long_plan.allowThrottle

    lead_one = self.sm['radarState'].leadOne
    lead_status = lead_one.status  # whether lead is valid
    if lead_status:
      lead_distance = lead_one.dRel  # relative distance in meters
      lead_velocity = lead_one.vRel  # relative velocity in m/s
    else:
      lead_distance = None
      lead_velocity = None


    can_sends = []

    apply_steer = 0
    ti_apply_steer = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * self.ccp.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, self.ccp)
      if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
        if CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * self.ccp.TI_STEER_MAX))
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                    CS.out.steeringTorque, self.ccp)
    self.apply_steer_last = apply_steer
    self.ti_apply_steer_last = ti_apply_steer

    if self.CP.flags & MazdaFlags.GEN1:
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if self.CP.flags & MazdaFlags.RADAR_INTERCEPTOR:
        hold = False
        if CS.out.standstill:
          hold = self.hold_timer.active()
        else:
          self.hold_timer.reset()

        if CC.longActive:
          if "ACCEL_CMD" in CS.crz_info:
            raw_acc_output = CC.actuators.accel * 1150
            raw_acc_output = max(-1000, min(raw_acc_output, 1000))

            if self.params.get_bool("BlendedACC"):
              if self.params_memory.get_int("CEStatus"):
                self.acc_filter.update_alpha(abs(raw_acc_output-self.filtered_acc_last)/1000)
                filtered_acc_output = int(self.acc_filter.update(raw_acc_output))
              else:
                # we want to use the stock value in this case but we need a smooth transition.
                self.acc_filter.update_alpha(abs(CS.crz_info["ACCEL_CMD"]-self.filtered_acc_last)/1000)
                filtered_acc_output = int(self.acc_filter.update(CS.crz_info["ACCEL_CMD"]))

              CS.crz_info["ACCEL_CMD"] = int(filtered_acc_output)
              self.filtered_acc_last = filtered_acc_output
            else:
              CS.crz_info["ACCEL_CMD"] = int(raw_acc_output)

        if self.frame % 2 == 0:
          can_sends.extend(mazdacan.create_radar_command(self.packer, self.frame, CC.longActive, CS, hold))
    # GEN2
    else:
      target_accel = CC.actuators.accel
      p = self.ccp.long_params

      # Step on brakes some more below ~15 mph
      if CS.out.vEgo < p.brake_overboost_threshold and target_accel < 0:
        # At 0 m/s = 2x multiplier
        # At 6 m/s = 1x multiplier
        brake_mult = p.brake_overboost_multiplier - (CS.out.vEgo / p.brake_overboost_threshold)
        target_accel *= brake_mult

      target_accel = max(p.accel_min, target_accel)
      raw_acc_output = _compute_can_cmd(target_accel, CS.out.vEgo, p, frogpilot_toggles)
      OPlong = (self.params.get_bool("ExperimentalLongitudinalEnabled") and CC.longActive)

      if OPlong:
        # Verify CS.acc has the necessary key before attempting ANY modification
        if "ACCEL_CMD" in CS.acc:
          if self.params.get_bool("BlendedACC"):
            CEStatus = self.params_memory.get_int("CEStatus")

            # Conditional Experimental Mode is active when status >= 2.
            # 1 is force disabled, 0 is default/inactive.
            if CEStatus >= 2:
              if self.blend_coeff < 1.0:
                self.blend_coeff += (DT_CTRL / self.transition_time)
                self.blend_coeff = min(1.0, self.blend_coeff)
            else:
              if self.blend_coeff > 0.0:
                self.blend_coeff -= (DT_CTRL / self.transition_time)
                self.blend_coeff = max(0.0, self.blend_coeff)

            # Apply blending if there's any OP contribution
            if self.blend_coeff > 0:
              blended_acc_output = (self.blend_coeff * raw_acc_output) + ((1 - self.blend_coeff) * CS.acc["ACCEL_CMD"])
              CS.acc["ACCEL_CMD"] = int(blended_acc_output)

            self.transition_time = (0.045455 * CS.out.vEgo) + 0.5 # Ramp transition time: ~0.5s at 0mph, ~1.6s at 55mph
            self.distance_last = CS.distance_setting

          else:
            # Pure OpenPilot Control
            CS.acc["ACCEL_CMD"] = int(raw_acc_output)

      resume = False
      hold = False
      if Timer.interval(2): # send ACC command at 50hz
        """
        Without this hold/resum logic, the car will only stop momentarily.
        It will then start creeping forward again. This logic allows the car to
        apply the electric brake to hold the car. The hold delay also fixes a
        bug with the stock ACC where it sometimes will apply the brakes too early
        when coming to a stop.
        """
        if CS.out.standstill: # if we're stopped
          if not self.hold_delay.active(): # and we have been stopped for more than hold_delay duration. This prevents a hard brake if we aren't fully stopped.
            if ((CC.cruiseControl.resume and CC.actuators.longControlState != LongCtrlState.stopping) or
                CC.cruiseControl.override or CS.out.gasPressed or
                (CC.actuators.longControlState == LongCtrlState.starting) or CS.acc.get("RESUME", False)): # if we are resuming or overriding, we want to release the brake
              self.resume_timer.reset() # reset the resume timer so its active
            else: # otherwise we're holding
              hold = self.hold_timer.active() # hold for 6s. This allows the electric brake to hold the car.

        else: # if we're moving
          self.hold_timer.reset() # reset the hold timer so its active when we stop
          self.hold_delay.reset() # reset the hold delay

        resume = self.resume_timer.active() # stay on for 0.5s to release the brake. This allows the car to move.
        if CS.out.vEgo < 1.0 and CS.acc.get("ACCEL_CMD", 0) > 2000:
          resume = True
          hold = False

        # Only send if the "ACCEL_CMD" key exists, implying we have valid stock data to mirror or modify
        if "ACCEL_CMD" in CS.acc:
          can_sends.append(mazdacan.create_acc_cmd(self.packer, CS.acc, hold, resume))

    # send steering command
    can_sends.extend(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_steer, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = apply_steer / self.ccp.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    Timer.tick()
    return new_actuators, can_sends


