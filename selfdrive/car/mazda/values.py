from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


# Steer torque limits


@dataclass
class Gen2LongitudinalParams:
  coast_coeffs: list[float]
  gas_coeffs: list[float]
  brake_coeffs: list[float]
  gas_intercept: float
  brake_intercept: float
  coast_intercept: float
  handoff_deadzone: float


@dataclass
class Gen2LateralParams:
  # Interpolation tables for speed-varying sigmoid+linear torque model
  # torque = sigmoid(A * lat_accel) * B + lat_accel * C
  speed_bp: list[float]  # speed breakpoints (m/s)
  a_vals: list[float]    # sigmoid steepness at each BP
  b_vals: list[float]    # sigmoid scale at each BP
  c_vals: list[float]    # linear gain at each BP

class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1
  GEN2 = 2
  TORQUE_INTERCEPTOR = 4
  RADAR_INTERCEPTOR = 8
  NO_FSC = 16
  NO_MRCC = 32
  MANUAL_TRANSMISSION = 64

# Tuning block
GEN2_LONG_TUNING = {
  MazdaFlags.GEN2: Gen2LongitudinalParams(
    coast_coeffs = [-0.0061451631, -1.5537955207, 0.0001278820, 0.0348897677, -7.4311831963],
    coast_intercept = 0.0695481408,

    gas_coeffs = [132.7561769120, -4.1304582766, 0.1170502643, 77.2589186500, 3.4148216730, -0.0842671481, 0.0514865701, -0.0009911633, -0.0000104751],
    gas_intercept = 1966.0536716186,

    brake_coeffs = [90.5923286338, 6.2348616441, -21.8473173056, 1.3475794744, -0.1042297028],
    brake_intercept = 1894.9223449037,
    handoff_deadzone = 0.05
  )
}

# Lateral tuning: speed-varying sigmoid+linear torque model
# torque = sigmoid(A(v) * lat_accel) * B(v) + lat_accel * C(v)
# where A(v) = a0 + a1*v + a2*v^2, etc.
# Run perform_lateral_regression.py to generate speed-varying coefficients from rlogs
#
# MAZDA_3: A=15.38616, B=0.71899, C=0.15015
# CX-30:   A=4.68689,  B=0.79999, C=0.18244
GEN2_LATERAL_TUNING = {
  MazdaFlags.GEN2: Gen2LateralParams(
    # From 220-log regression (bin centers)
    speed_bp = [5.0, 9.5, 15.0, 21.5, 30.0],
    a_vals = [13.21, 1.10, 2.71, 1.02, 1.12],  # sigmoid steepness
    b_vals = [0.90, 1.37, 0.75, 1.55, 1.52],   # sigmoid scale
    c_vals = [0.05, 0.05, 0.12, 0.05, 0.05],   # linear gain (clamped min 0.05)
  )
}

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 1 # 100 Hz
    if CP.flags & MazdaFlags.GEN1:
      self.STEER_MAX = 600                # theoretical max_steer 2047
      self.STEER_DELTA_UP = 10             # torque increase per refresh
      self.STEER_DELTA_DOWN = 25           # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 40     # weight driver torque
      self.STEER_DRIVER_FACTOR = 1         # from dbc
      self.STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor

      self.TI_STEER_MAX = 600                # theoretical max_steer 2047
      self.TI_STEER_DELTA_UP = 6             # torque increase per refresh
      self.TI_STEER_DELTA_DOWN = 15           # torque decrease per refresh
      self.TI_STEER_DRIVER_ALLOWANCE = 15    # allowed driver torque before start limiting
      self.TI_STEER_DRIVER_MULTIPLIER = 40     # weight driver torque
      self.TI_STEER_DRIVER_FACTOR = 1         # from dbc
      self.TI_STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor
    if CP.flags & MazdaFlags.GEN2:
      self.STEER_MAX = 8000
      self.STEER_DELTA_UP = 45              # torque increase per refresh
      self.STEER_DELTA_DOWN = 80            # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 1400     # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 5      # weight driver torque
      self.STEER_DRIVER_FACTOR = 1           # from dbc
      self.STEER_ERROR_MAX = 3500            # max delta between torque cmd and torque motor

      # Load longitudinal tuning
      self.long_params = GEN2_LONG_TUNING.get(MazdaFlags.GEN2)
      # Load lateral tuning (speed-varying ABC coeffs)
      self.lateral_params = GEN2_LATERAL_TUNING.get(MazdaFlags.GEN2)

class TI_STATE:
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3

@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('mazda_2017', None))
  def init(self):
    if self.flags & MazdaFlags.GEN2:
      self.dbc_dict = dbc_dict('mazda_2019', None)
    elif self.flags & MazdaFlags.GEN1 and self.flags & MazdaFlags.RADAR_INTERCEPTOR:
      self.dbc_dict = dbc_dict('mazda_2017', 'mazda_radar')



class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5),
    flags=MazdaFlags.GEN1
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video_link="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs,
    flags=MazdaFlags.GEN1,
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-24")],
    MAZDA_CX5.specs,
    flags=MazdaFlags.GEN1,
  )
  MAZDA_3_2019 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2019-24")],
    MazdaCarSpecs(mass=3000 * CV.LB_TO_KG, wheelbase=2.725, steerRatio=18.8),
    flags=MazdaFlags.GEN2,
  )
  MAZDA_CX_30 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-30 2019-24")],
    MazdaCarSpecs(mass=3375 * CV.LB_TO_KG, wheelbase=2.814, steerRatio=15.5),
    flags=MazdaFlags.GEN2,
  )
  MAZDA_CX_50 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-50 2022-24")],
    MazdaCarSpecs(mass=3375 * CV.LB_TO_KG, wheelbase=2.814, steerRatio=15.5),
    flags=MazdaFlags.GEN2,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph
  TI_STEER_THRESHOLD = 6
  TI_DISABLE_SPEED = 0    # kph
  TI_ENABLE_SPEED = 0     # kph

class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4
  TURN_ON = 5


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.shiftByWire],
    )
  ],
)

DBC = CAR.create_dbc_map()
GEN1 = CAR.with_flags(MazdaFlags.GEN1)
GEN2 = CAR.with_flags(MazdaFlags.GEN2)
