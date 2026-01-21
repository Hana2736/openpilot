#!/usr/bin/env python3
import math
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import ACCEL_MIN

from openpilot.frogpilot.common.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED, THRESHOLD, params_memory, scale_threshold

class ConditionalExperimentalMode:
  # ===== CONDITIONAL EXPERIMENTAL MODE SPEED-BASED TUNING =====
  # Speed ranges: [0-35, 35-55, 55-70, 70+ mph]

  # FILTER TIME CONSTANTS (Lower = More responsive, Higher = Smoother)
  # [City, Urban Hwy, Rural Hwy, High Speed]
  FILTER_TIME_CURVES = [0.9, 0.8, 0.6, 0.5]    # Faster detection at highway speeds
  FILTER_TIME_LEADS = [0.9, 0.8, 0.7, 0.5]     # Less sensitive at 70+ mph for slow leads
  FILTER_TIME_LIGHTS = [0.9, 0.8, 0.75, 0.55]  # Less sensitive at 60+ mph for stoplights

  # HIGHWAY LIGHT DETECTION MULTIPLIERS
  # How much to increase model stop time at highway speeds
  LIGHT_BOOSTS = [1.0, 1.2, 1.045, 1.0]         # Slightly less sensitive at 60mph (5% reduction)
  LIGHT_SPEED_LOW = 50 * CV.MPH_TO_MS     # 50 mph threshold
  LIGHT_SPEED_HIGH = 60 * CV.MPH_TO_MS    # 60 mph threshold
  LIGHT_MAX_TIME = 9       # Balanced max time preserving city performance

  # ===== END TUNING PARAMETERS =====

  # Current active values
  FILTER_TIME_CURVE = 0.8
  FILTER_TIME_LEAD = 0.8
  FILTER_TIME_LIGHT = 0.8
  LIGHT_BOOST_LOW = 1.15
  LIGHT_BOOST_HIGH = 1.2

  @staticmethod
  def get_speed_based_param(speed_mph, param_array):
    """Get parameter value based on current speed using smooth interpolation between breakpoints [0, 35, 55, 70]"""
    return interp(speed_mph, [0, 35, 55, 70], param_array)

  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    # Faster filters with hysteresis for better responsiveness
    self.curvature_filter = FirstOrderFilter(0, self.FILTER_TIME_CURVE, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, self.FILTER_TIME_LEAD, DT_MDL)
    self.stop_light_filter = FirstOrderFilter(0, self.FILTER_TIME_LIGHT, DT_MDL)

    self.curve_detected = False
    self.experimental_mode = False
    self.stop_light_detected = False
    self.prev_experimental_mode = False  # For hysteresis

  def update(self, v_ego, sm, frogpilot_toggles):
    if frogpilot_toggles.experimental_mode_via_press:
      self.status_value = params_memory.get_int("CEStatus")
    else:
      self.status_value = 0

    if self.status_value not in {1, 2} and not sm["carState"].standstill:
      self.update_conditions(v_ego, sm, frogpilot_toggles)
      new_experimental_mode = self.check_conditions(v_ego, sm, frogpilot_toggles)

      # Add hysteresis to prevent rapid toggling
      if new_experimental_mode and not self.prev_experimental_mode:
        # Require weaker conditions to turn on
        hysteresis_factor = 0.9
      elif not new_experimental_mode and self.prev_experimental_mode:
        # Require stronger conditions to turn off
        hysteresis_factor = 1.2
      else:
        hysteresis_factor = 1.0

      # Apply hysteresis to key conditions
      if hasattr(self, 'slow_lead_detected'):
        self.slow_lead_detected = self.slow_lead_detected if hysteresis_factor == 1.0 else (self.slow_lead_filter.x >= scale_threshold(v_ego) * hysteresis_factor)
      if hasattr(self, 'curve_detected'):
        self.curve_detected = self.curve_detected if hysteresis_factor == 1.0 else (self.curvature_filter.x >= THRESHOLD * hysteresis_factor)

      self.experimental_mode = self.check_conditions(v_ego, sm, frogpilot_toggles)
      self.prev_experimental_mode = self.experimental_mode
      params_memory.put_int("CEStatus", self.status_value if self.experimental_mode else 0)
    else:
      self.experimental_mode = self.status_value == 2 or sm["carState"].standstill and self.experimental_mode and self.frogpilot_planner.model_stopped
      self.stop_light_detected &= self.status_value not in {1, 2}
      self.stop_light_filter.x = 0

  def check_conditions(self, v_ego, sm, frogpilot_toggles):
    below_speed = frogpilot_toggles.conditional_limit > v_ego >= 1 and not self.frogpilot_planner.frogpilot_following.following_lead
    below_speed_with_lead = frogpilot_toggles.conditional_limit_lead > v_ego >= 1 and self.frogpilot_planner.frogpilot_following.following_lead
    if below_speed or below_speed_with_lead:
      self.status_value = 3 if self.frogpilot_planner.frogpilot_following.following_lead else 4
      return True

    desired_lane = self.frogpilot_planner.lane_width_left if sm["carState"].leftBlinker else self.frogpilot_planner.lane_width_right
    lane_available = desired_lane >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.conditional_signal_lane_detection
    if v_ego < frogpilot_toggles.conditional_signal and (sm["carState"].leftBlinker or sm["carState"].rightBlinker) and not lane_available:
      self.status_value = 5
      return True

    approaching_maneuver = sm["frogpilotNavigation"].approachingIntersection or sm["frogpilotNavigation"].approachingTurn
    if frogpilot_toggles.conditional_navigation and approaching_maneuver and (frogpilot_toggles.conditional_navigation_lead or not self.frogpilot_planner.frogpilot_following.following_lead):
      self.status_value = 6 if sm["frogpilotNavigation"].approachingIntersection else 7
      return True

    if frogpilot_toggles.conditional_curves and self.curve_detected and (frogpilot_toggles.conditional_curves_lead or not self.frogpilot_planner.frogpilot_following.following_lead):
      self.status_value = 8
      return True

    if frogpilot_toggles.conditional_lead and self.slow_lead_detected and v_ego <= 35.31:
      self.status_value = 9 if self.frogpilot_planner.lead_one.vLead < 1 else 10
      return True

    if frogpilot_toggles.conditional_model_stop_time != 0 and self.stop_light_detected:
      self.status_value = 11 if not self.frogpilot_planner.frogpilot_vcruise.forcing_stop else 12
      return True

    if self.frogpilot_planner.frogpilot_vcruise.slc.experimental_mode:
      self.status_value = 13
      return True

    return False

  def update_conditions(self, v_ego, sm, frogpilot_toggles):
    self.curve_detection(v_ego, frogpilot_toggles)
    self.slow_lead(v_ego, frogpilot_toggles)
    self.stop_sign_and_light(v_ego, sm, frogpilot_toggles.conditional_model_stop_time)

  def curve_detection(self, v_ego, frogpilot_toggles):
    self.curvature_filter.update(self.frogpilot_planner.road_curvature_detected or self.frogpilot_planner.driving_in_curve)
    self.curve_detected = self.curvature_filter.x >= THRESHOLD and v_ego > CRUISING_SPEED

  def slow_lead(self, v_ego, frogpilot_toggles):
    if self.frogpilot_planner.tracking_lead:
      lead = self.frogpilot_planner.lead_one
      lead_distance = lead.dRel
      relative_speed = v_ego - lead.vLead

      # Physics-based safe stop time calculation
      wanted_stop_time = self.get_safe_stop_time(frogpilot_toggles.conditional_model_stop_time)
      safe_approach_dist = self.get_safe_distance(relative_speed, wanted_stop_time * (2.0/3.0))

      # Slower lead detection: are we closing quickly on them?
      closing_quickly = relative_speed > (CRUISING_SPEED * 0.75)  # ~8.5 mph faster than them
      close_proximity = lead_distance < safe_approach_dist
      slower_lead = closing_quickly and close_proximity and frogpilot_toggles.conditional_slower_lead

      # Stopped lead detection with physics-based distance
      safe_stopped_dist = self.get_safe_distance(relative_speed, wanted_stop_time)
      lead_is_stopped = lead.vLead < 3  # 3 m/s tolerance for stopped detection
      lead_is_in_range = lead_distance < safe_stopped_dist
      stopped_lead = lead_is_stopped and lead_is_in_range and frogpilot_toggles.conditional_stopped_lead

      # Adjust threshold based on lead probability for vision-only accuracy
      lead_prob = getattr(lead, 'modelProb', 1.0)
      lead_threshold = scale_threshold(v_ego) * (1.0 + 0.2 * (1.0 - lead_prob))

      self.slow_lead_filter.update(slower_lead or stopped_lead)
      self.slow_lead_detected = self.slow_lead_filter.x >= lead_threshold
    else:
      self.slow_lead_filter.x = 0
      self.slow_lead_detected = False

  def stop_sign_and_light(self, v_ego, sm, model_time):
    if not sm["frogpilotCarState"].trafficModeEnabled:
      speed_mph = v_ego * CV.MS_TO_MPH

      # Interp for smooth scaling in 35-45 mph
      bp = [0, 35, 45]
      low_filter_time = 0.0
      tuned_filter_time_curves = self.FILTER_TIME_CURVES[1]
      tuned_filter_time_leads = self.FILTER_TIME_LEADS[1]
      tuned_filter_time_lights = self.FILTER_TIME_LIGHTS[1]
      low_boost = 1.0
      tuned_boost = self.LIGHT_BOOSTS[1]
      low_cap_factor = 0.0
      tuned_cap_factor = 1.0

      filter_time_curves = interp(speed_mph, bp, [low_filter_time, low_filter_time, tuned_filter_time_curves])
      filter_time_leads = interp(speed_mph, bp, [low_filter_time, low_filter_time, tuned_filter_time_leads])
      filter_time_lights = interp(speed_mph, bp, [low_filter_time, low_filter_time, tuned_filter_time_lights])
      light_boost = interp(speed_mph, bp, [low_boost, low_boost, tuned_boost])
      cap_factor = interp(speed_mph, bp, [low_cap_factor, low_cap_factor, tuned_cap_factor])

      # Update filter times with interp
      self.curvature_filter = FirstOrderFilter(self.curvature_filter.x, filter_time_curves, DT_MDL)
      self.slow_lead_filter = FirstOrderFilter(self.slow_lead_filter.x, filter_time_leads, DT_MDL)
      self.stop_light_filter = FirstOrderFilter(self.stop_light_filter.x, filter_time_lights, DT_MDL)

      # Disable stoplight detection at very high speeds to prevent false positives
      if speed_mph > 75:
        self.stop_light_filter.x = 0
        self.stop_light_detected = False
        return

      model_length = self.frogpilot_planner.model_length

      # Physics-based safe stop distance calculation
      safe_stop_dist = self.get_safe_distance(v_ego, self.get_safe_stop_time(model_time))

      # Adjust model time with interp boost and gradual cap
      adjusted_model_time = model_time * light_boost
      if cap_factor > 0:
        adjusted_model_time = min(adjusted_model_time, self.LIGHT_MAX_TIME * cap_factor + model_time * (1 - cap_factor))

      model_stopping = model_length < max(v_ego * adjusted_model_time, safe_stop_dist)

      self.stop_light_filter.update(self.frogpilot_planner.model_stopped or model_stopping)
      light_detected = self.stop_light_filter.x >= THRESHOLD**2

      should_stop_for_light = light_detected

      # When following a lead, only trigger if stop is distinct from lead position
      if self.frogpilot_planner.tracking_lead:
        lead = self.frogpilot_planner.lead_one
        lead_is_stopped = lead.vLead < 3.0
        # Stop is distinct if model stop point is beyond lead by more than 3m
        stop_is_distinct = model_length < (lead.dRel - 3.0)
        should_stop_for_light = light_detected and (lead_is_stopped or stop_is_distinct)

      self.stop_light_detected = should_stop_for_light
    else:
      self.stop_light_filter.x = 0
      self.stop_light_detected = False

  def get_safe_stop_time(self, raw_value):
    """
    Stop time for stopped leads and red light/stop sign.
    Return 10 seconds if we have an invalid config to avoid rear-ending someone.
    """
    fallback_value = 10

    try:
      if raw_value is None:
        return fallback_value

      val = float(raw_value)

      if math.isnan(val) or math.isinf(val):
        return fallback_value

      # Guard against template val (0 or 1)
      if val <= 1.5:
        return fallback_value

      return val

    except (ValueError, TypeError):
      return fallback_value

  def get_safe_distance(self, velocity, time_threshold):
    """
    Return the safe stopping distance based on physics.
    If we can't stop in the target time at max decel, brake earlier.
    """
    if velocity <= 0:
      return 0

    # Use car's max braking with wiggle room
    safe_decel = abs(ACCEL_MIN) * 0.90

    # Time-based distance
    d_time = velocity * time_threshold

    # Physics limit: v^2 / (2 * a)
    d_physics = (velocity ** 2) / (2 * safe_decel)

    # Return the larger distance so we brake early if needed
    return max(d_time, d_physics)
