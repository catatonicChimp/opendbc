import random
import re
import unittest
from types import SimpleNamespace

from opendbc.car import DT_CTRL, structs
from opendbc.car.volkswagen import mqbcan
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import CarController, HCAMitigation
from opendbc.car.volkswagen.values import CAR, DBC, CarControllerParams as CCP, WMI, match_fw_to_car_fuzzy
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"

class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): {radar_fw}}
              matches = match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"


class TestVolkswagenMqbGasOverride(unittest.TestCase):
  @staticmethod
  def _acc_status_sequence(frames):
    CP = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK7, {bus: {} for bus in range(8)}, [], True, False, False)
    controller = CarController(DBC[CP.carFingerprint], CP)
    statuses = []
    for enabled, long_active, gas_pressed in frames:
      CC = structs.CarControl(enabled=enabled, longActive=long_active)
      controller.update_gas_override(SimpleNamespace(out=SimpleNamespace(gasPressed=gas_pressed)), CC)
      statuses.append(mqbcan.acc_control_value(True, False, CC.longActive, controller.gas_override))
    return statuses

  def test_gas_release_has_no_standby_frame(self):
    # longActive lags gasPressed by a frame on both press and release. A single ACC_STANDBY (2) frame between
    # ACC_OVERRIDE (4) and ACC_ACTIVE (3) makes TSK_06 drop to standby and then fault
    frames = [
      (True, True, False),   # engaged
      (True, True, True),    # gas pressed, longActive not yet updated
      (True, False, True),   # override
      (True, False, True),
      (True, False, False),  # gas released, longActive not yet updated
      (True, True, False),   # engaged again
    ]
    self.assertEqual(self._acc_status_sequence(frames), [3, 3, 4, 4, 4, 3])

  def test_override_requires_gas(self):
    # Long paused for reasons other than the gas pedal is not an override
    self.assertEqual(self._acc_status_sequence([(True, True, False), (True, False, False), (True, True, False)]), [3, 2, 3])

  def test_override_ends_on_disengage(self):
    self.assertEqual(self._acc_status_sequence([(True, False, True), (False, False, False), (False, False, True)]), [4, 2, 2])
