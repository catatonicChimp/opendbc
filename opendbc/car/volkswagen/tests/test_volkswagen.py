import random
import re
import unittest

from opendbc.can import CANPacker

from opendbc.can.dbc import DBC as DBCParsed
from opendbc.car import Bus, DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen import mqbcan
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.values import CAR, DBC, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
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

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"


class TestVolkswagenMqbSetSpeedMarker(unittest.TestCase):
  @staticmethod
  def hud(status, reached=False):
    packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK7][Bus.pt])
    dat = int.from_bytes(bytes(mqbcan.create_acc_hud_control(packer, 0, status, 50., 0, 1, speed_reached=reached)[1]), 'little')
    sigs = DBCParsed('vw_mqb').name_to_msg["ACC_02"].sigs
    return tuple((dat >> sigs[name].start_bit) & 1 for name in ("ACC_Tachokranz", "ACC_Wunschgeschw_erreicht"))

  def test_marker_on_while_engaged(self):
    # Stock radar keeps the speedometer set-speed marker on while engaged (3) or overridden (4), off otherwise
    for status in (3, 4):
      self.assertEqual(self.hud(status)[0], 1)
    for status in (0, 2, 6):
      self.assertEqual(self.hud(status)[0], 0)
    self.assertEqual(self.hud(3, reached=True)[1], 1)
    self.assertEqual(self.hud(3)[1], 0)

  def test_reached_matches_stock(self):
    # (v_ego, set speed, stock ACC_Wunschgeschw_erreicht) from stock ACC, in order. Stock also waits ~2 s after
    # engaging or a set speed change before setting it, which isn't modelled
    for trace in ([(39.7, 40, 1), (39.5, 41.92, 1), (39.3, 41.92, 1), (34.6, 41.92, 0)],
                  [(47.1, 40, 0), (45.8, 40, 0), (44.6, 49.92, 0)]):
      reached = False
      for v, set_speed, stock in trace:
        reached = mqbcan.set_speed_reached(reached, True, v, set_speed)
        self.assertEqual(reached, bool(stock), (v, set_speed))
    self.assertTrue(mqbcan.set_speed_reached(False, True, 37.4, 40))
    self.assertFalse(mqbcan.set_speed_reached(True, False, 40, 40))
