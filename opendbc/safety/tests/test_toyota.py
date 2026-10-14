#!/usr/bin/env python3
import os
import numpy as np
import random
import unittest
import itertools

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

TOYOTA_COMMON_TX_MSGS = [[0x2E4, 0], [0x191, 0], [0x412, 0], [0x343, 0], [0x1D2, 0]]  # LKAS + LTA + ACC & PCM cancel cmds
TOYOTA_SECOC_TX_MSGS = [[0x131, 0], [0x183, 0]] + TOYOTA_COMMON_TX_MSGS
# LionDriver reference build: libsafety compiled with LIONDRIVER_REFERENCE (see libsafety_py)
LIONDRIVER_REFERENCE = os.environ.get("LIONDRIVER_REFERENCE") == "1"

TOYOTA_COMMON_LONG_TX_MSGS = [[0x283, 0],  # PRE_COLLISION, zero payload only
                              [0x750, 0]]  # radar diagnostic address
if not LIONDRIVER_REFERENCE:
  TOYOTA_COMMON_LONG_TX_MSGS.append([0x411, 0])  # PCS_HUD, not in the reference build (TSR-705 D-1)


class TestToyotaSafetyBase(common.CarSafetyTest, common.LongitudinalAccelSafetyTest):

  DBC = "toyota_nodsu_pt_generated"
  SAFETY_MODEL = CarParams.SafetyModel.toyota

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x343)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191, 0x343]}
  EPS_SCALE = 73

  def _torque_meas_msg(self, torque: int, driver_torque: int | None = None):
    values = {"STEER_TORQUE_EPS": (torque / self.EPS_SCALE) * 100.}
    if driver_torque is not None:
      values["STEER_TORQUE_DRIVER"] = driver_torque
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  # Both torque and angle safety modes test with each other's steering commands
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": steer_req}
    return self.packer.make_can_msg_safety("STEERING_LKA", 0, values)

  def _angle_meas_msg(self, angle: float, steer_angle_initializing: bool = False):
    # This creates a steering torque angle message. Not set on all platforms,
    # relative to init angle on some older TSS2 platforms. Only to be used with LTA
    values = {"STEER_ANGLE": angle, "STEER_ANGLE_INITIALIZING": int(steer_angle_initializing)}
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    return self._lta_msg(int(enabled), int(enabled), angle, torque_wind_down=100 if enabled else 0)

  def _lta_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd, "TORQUE_WIND_DOWN": torque_wind_down}
    return self.packer.make_can_msg_safety("STEERING_LTA", 0, values)

  def _accel_msg_343(self, accel, cancel_req=0):
    values = {"ACCEL_CMD": accel, "CANCEL_REQ": cancel_req}
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_343(accel, cancel_req)

  def _speed_msg(self, speed, quality_flag=True):
    values = {("WHEEL_SPEED_%s" % n): speed * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    if not quality_flag:
      values |= {"WHEEL_SPEED_%s_FAULT" % n: 1.0 for n in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  def _user_gas_msg(self, gas):
    cruise_active = self.safety.get_controls_allowed()
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": cruise_active}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ACTIVE": enable}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def test_pcs_hud(self):
    # LionDriver TSR-705 decision D-1: PCS_HUD is not accepted in the reference build
    expected = [0x411, 0] in self.TX_MSGS
    if LIONDRIVER_REFERENCE:
      self.assertFalse(expected)
    self.assertEqual(expected, self._tx(libsafety_py.make_CANPacket(0x411, 0, b"\x00" * 8)))

  def test_pcs_and_dsu_messages_blocked(self):
    # LionDriver TSR-705: frames used by the stock PCS function or sent only by a DSU are never forwarded from the SoC
    blocked = [(0x344, 0, 8), (0x2E6, 0, 8), (0x2E7, 0, 8), (0x33E, 0, 7), (0x365, 0, 7), (0x366, 0, 7), (0x4CB, 0, 8),
               (0x128, 1, 6), (0x141, 1, 4), (0x160, 1, 8), (0x161, 1, 7), (0x470, 1, 4)]
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for addr, bus, length in blocked:
        self.assertFalse(self._tx(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length)), hex(addr))

  def _tx(self, msg):
    # LionDriver TSR-518: content tests send many frames at one timer value; clear the TX rate windows so that they
    # check the content limits only. The rate tests below use _tx_rated
    self.safety.reset_tx_rate_state()
    return super()._tx(msg)

  def _tx_rated(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _tx_rate_rows(self):
    # LionDriver TSR-518 (DN-01): reference build, non-SecOC: (addr, nominal Hz, burst, a frame that passes the content checks)
    if not LIONDRIVER_REFERENCE or self.SAFETY_PARAM & ToyotaSafetyFlags.SECOC:
      return []
    rows = [(0x2E4, 100, 5, lambda: self._torque_cmd_msg(0, steer_req=0)),
            (0x191, 50, 3, lambda: libsafety_py.make_CANPacket(0x191, 0, b"\x00" * 8)),
            (0x412, 5, 4, lambda: libsafety_py.make_CANPacket(0x412, 0, b"\x00" * 8)),
            (0x1D2, 1, 2, lambda: libsafety_py.make_CANPacket(0x1D2, 0, b"\x00" * 8)),
            (0x343, 33, 3, lambda: self._accel_msg_343(0, cancel_req=1))]
    if [0x283, 0] in self.TX_MSGS:
      rows.append((0x283, 1, 2, lambda: libsafety_py.make_CANPacket(0x283, 0, b"\x00" * 7)))
    if [0x750, 0] in self.TX_MSGS:
      rows.append((0x750, 5, 2, lambda: libsafety_py.make_CANPacket(0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00")))
    return rows

  @staticmethod
  def _tx_rate_budget(freq, burst):
    return -(-freq * 3 // 10) + 1 + burst

  def test_tx_rate_every_whitelisted_id_has_a_budget(self):
    # every TX-whitelisted ID of the reference build is rate-checked (DN-01 T6)
    rows = self._tx_rate_rows()
    if not rows:
      raise unittest.SkipTest("reference build, non-SecOC")
    self.assertEqual({addr for addr, *_ in rows}, {addr for addr, bus in self.TX_MSGS})

  def test_tx_rate_flood_rejected(self):
    # beyond its budget in one window, a frame is rejected; the budget is ceil(f * 0.25 * 1.2) + 1 + burst (DN-01 T2)
    for addr, freq, burst, make in self._tx_rate_rows():
      with self.subTest(addr=hex(addr)):
        self.assertTrue(self._tx(make()), "frame must pass the content checks")
        self._reset_safety_hooks()
        self.safety.set_timer(1000000)
        budget = self._tx_rate_budget(freq, burst)
        results = [self._tx_rated(make()) for _ in range(budget + 5)]
        self.assertEqual(results, [True] * budget + [False] * 5)
        self.assertFalse(self.safety.get_safety_tx_rate_exceeded())  # controls were not allowed: nothing to revoke

  def test_tx_rate_window_slides(self):
    # the window is made of two half-windows, so a burst cannot double the budget at a window edge (DN-01 §4.2)
    for addr, freq, burst, make in self._tx_rate_rows():
      with self.subTest(addr=hex(addr)):
        self._reset_safety_hooks()
        budget = self._tx_rate_budget(freq, burst)
        t0 = 1000000
        self.safety.set_timer(t0)
        self.assertTrue(self._tx_rated(make()))
        self.safety.set_timer(t0 + 240000)
        for _ in range(budget - 1):
          self.assertTrue(self._tx_rated(make()))
        # 15 ms later: a fixed 250 ms window would have started afresh; the sliding window still holds the burst
        self.safety.set_timer(t0 + 255000)
        self.assertFalse(self._tx_rated(make()))

  def test_tx_rate_host_cadence_passes(self):
    # 10 s of the host's cadence, including on-change LKAS_HUD frames: nothing rejected, authority kept (DN-01 T1)
    rows = {addr: make for addr, _, _, make in self._tx_rate_rows()}
    if not rows:
      raise unittest.SkipTest("reference build, non-SecOC")
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    for frame in range(1000):
      self.safety.set_timer(1000000 + frame * 10000)
      sends = [0x2E4]
      if frame % 2 == 0:
        sends.append(0x191)
      if frame % 3 == 0:
        sends.append(0x343)
      if frame % 20 == 0 or frame % 97 in (0, 1, 2):  # 5 Hz, plus alert changes
        sends.append(0x412)
      for addr in sends:
        self.assertTrue(self._tx_rated(rows[addr]()), (frame, hex(addr)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_rate_persistent_excess_revokes(self):
    # STEERING_LKA at 2x its rate: the third rejected frame in a half-window revokes authority and is reported (DN-01 T3)
    rows = {addr: make for addr, _, _, make in self._tx_rate_rows()}
    if not rows:
      raise unittest.SkipTest("reference build, non-SecOC")
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    first_rejection = revoked = None
    for i in range(200):
      t = 1000000 + i * 5000  # 200 Hz
      self.safety.set_timer(t)
      if not self._tx_rated(rows[0x2E4]()) and first_rejection is None:
        first_rejection = t
      if not self.safety.get_controls_allowed():
        revoked = t
        break
    self.assertIsNotNone(revoked)
    self.assertTrue(self.safety.get_safety_tx_rate_exceeded())
    self.assertLessEqual(revoked - first_rejection, 100000)  # TSR-518: <= 0.1 s from the first rejected frame
    self.assertLessEqual(revoked - 1000000, 200000)          # DN-01 §4.2: about 0.19 s from the onset at 2x

    # the report is cleared by the next safety mode change
    self._reset_safety_hooks()
    self.assertFalse(self.safety.get_safety_tx_rate_exceeded())

  def test_tx_rate_no_reengagement_while_excess_persists(self):
    # SWSR-518 amendment (WP-W-03 ATC-20, RR-2026-009 F-34): after a revocation, a cruise rising edge does not grant
    # authority again until one full 250 ms window has passed without a rejected frame
    rows = {addr: make for addr, _, _, make in self._tx_rate_rows()}
    if not rows:
      raise unittest.SkipTest("reference build, non-SecOC")

    def engage(t):
      # a fresh cruise rising edge with every RX check valid at that moment
      self.safety.set_timer(t)
      self._rx_all_checked_msgs()
      self._rx(self._pcm_status_msg(True))
      return self.safety.get_controls_allowed()

    self._reset_safety_hooks()
    t = 1000000
    self.assertTrue(engage(t))
    last_reject = None
    while self.safety.get_controls_allowed():  # STEERING_LKA at 2x its rate until revoked
      t += 5000
      self.safety.set_timer(t)
      if not self._tx_rated(rows[0x2E4]()):
        last_reject = t
    self.assertTrue(self.safety.get_safety_tx_rate_exceeded())

    # the flood goes on: every new engagement is refused
    for _ in range(10):
      t += 5000
      self.safety.set_timer(t)
      self.assertFalse(self._tx_rated(rows[0x2E4]()))
      last_reject = t
      self.assertFalse(engage(t))

    # the flood stops: still refused within one window of the last rejected frame, granted after it
    self.assertFalse(engage(last_reject + 249999))
    self.assertTrue(engage(last_reject + 250000))
    self.assertTrue(self.safety.get_safety_tx_rate_exceeded())  # the report stays latched until the mode changes

    # the block only follows a revocation: rejections while authority was never granted do not block engagement
    self._reset_safety_hooks()
    t = 3000000
    self.safety.set_timer(t)
    for _ in range(40):
      self._tx_rated(rows[0x2E4]())
    self.assertTrue(engage(t + 1))

  def test_no_grant_after_mode_set_without_new_engagement(self):
    # LionDriver SWSR-301/SWSR-311 (ATC-26, back-to-back finding B2B-06): a safety mode set while CRUISE_ACTIVE stays 1
    # must not let the next PCM_CRUISE frame grant authority; a new engagement (cruise seen off, then on) is needed
    if not LIONDRIVER_REFERENCE or self.SAFETY_PARAM & (ToyotaSafetyFlags.SECOC | ToyotaSafetyFlags.ALT_BRAKE):
      raise unittest.SkipTest("reference build, non-SecOC, BRAKE_MODULE 0x226")
    self.safety.set_timer(1000000)
    self._rx_all_checked_msgs()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.assertFalse(self.safety.get_controls_allowed())
    # every other RX check is received again, then PCM_CRUISE with CRUISE_ACTIVE still 1
    for msg in (self._speed_msg(0), self._user_brake_msg(False), self._torque_meas_msg(0),
                *[m for m, _ in self._reference_rx_check_msgs()]):
      self.assertTrue(self._rx(msg))
    for _ in range(3):
      self._rx(self._pcm_status_msg(True))
      self.assertFalse(self.safety.get_controls_allowed())

    # a new engagement grants authority
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_rate_catch_up_after_stall(self):
    # after a host stall shorter than the heartbeat limit (0.3 s), the queued frames arrive at once: all pass (DN-01 T4)
    rows = {addr: make for addr, _, _, make in self._tx_rate_rows()}
    if not rows:
      raise unittest.SkipTest("reference build, non-SecOC")
    for gap_ms in (50, 100, 150, 200, 240, 290):
      with self.subTest(gap_ms=gap_ms):
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(True)
        t = 1000000
        for _ in range(100):  # 1 s at 100 Hz
          self.safety.set_timer(t)
          self.assertTrue(self._tx_rated(rows[0x2E4]()))
          t += 10000
        t += gap_ms * 1000
        self.safety.set_timer(t)
        for _ in range(gap_ms // 10):
          self.assertTrue(self._tx_rated(rows[0x2E4]()))
        self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_rate_rejected_frame_does_not_update_limits(self):
    # a frame over budget is rejected before the torque limit checks see it (DN-01 T5)
    rows = {addr: (freq, burst) for addr, freq, burst, _ in self._tx_rate_rows()}
    if 0x2E4 not in rows or self.SAFETY_PARAM & ToyotaSafetyFlags.LTA:
      raise unittest.SkipTest("reference build, LKA torque control")
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1000000)
    for _ in range(self._tx_rate_budget(*rows[0x2E4])):
      self.assertTrue(self._tx_rated(self._torque_cmd_msg(0, steer_req=0)))
    self.assertFalse(self._tx_rated(self._torque_cmd_msg(15, steer_req=1)))  # over budget
    # if the rejected frame had set the last torque to 15, a step to 30 would pass the rate-up limit of 15
    self.assertFalse(self._tx(self._torque_cmd_msg(30, steer_req=1)))

  def setUp(self):
    super().setUp()
    # LionDriver TSR-306: controls are only granted when every RX check is valid, as on a live bus
    self._rx_all_checked_msgs()

  def _rx_all_checked_msgs(self):
    # PCM_CRUISE before the gas message: on non-SecOC cars both are PCM_CRUISE, and the gas message leaves gas released
    for msg in (self._speed_msg(0), self._user_brake_msg(False), self._pcm_status_msg(False),
                self._user_gas_msg(False), self._torque_meas_msg(0), *[m for m, _ in self._reference_rx_check_msgs()]):
      self.assertTrue(self._rx(msg))

  def _reference_rx_check_msgs(self):
    # LionDriver TSR-110, TSR-401: RX checks the reference build adds on non-SecOC cars with BRAKE_MODULE, with their nominal frequency in Hz
    if not LIONDRIVER_REFERENCE or self.SAFETY_PARAM & (ToyotaSafetyFlags.SECOC | ToyotaSafetyFlags.ALT_BRAKE):
      return []
    return [(self._eps_status_msg(5), 25),
            (self._pcm_cruise_2_msg(main_on=True), 33)]

  def _pcm_cruise_2_msg(self, main_on: bool):
    return self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, {"MAIN_ON": int(main_on)})

  def test_cruise_active_with_main_off(self):
    # LionDriver TSR-405(c): CRUISE_ACTIVE with MAIN_ON = 0 is refused at once when cruise becomes active, and revokes
    # authority after the 150 ms tolerance when main goes off while engaged
    if not self._reference_rx_check_msgs():
      raise unittest.SkipTest("reference build, non-SecOC cars with PCM_CRUISE_2")
    tolerance_us = 150000

    # cruise becomes active while main is off: no engagement
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx_all_checked_msgs()
    self.assertTrue(self._rx(self._pcm_cruise_2_msg(main_on=False)))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

    # main goes off while engaged: tolerated up to the tolerance, revoked after it
    for elapsed_us, allowed in ((tolerance_us, True), (tolerance_us + 1, False)):
      with self.subTest(elapsed_us=elapsed_us):
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self._rx_all_checked_msgs()
        self._rx(self._pcm_status_msg(True))
        self.assertTrue(self.safety.get_controls_allowed())
        self._rx(self._pcm_cruise_2_msg(main_on=False))
        self.assertTrue(self.safety.get_controls_allowed())
        self.safety.set_timer(elapsed_us)
        self._rx(self._pcm_status_msg(True))
        self.assertEqual(allowed, self.safety.get_controls_allowed())

    # the disagreement ends when main is back on: the tolerance starts again on the next disagreement
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx_all_checked_msgs()
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_cruise_2_msg(main_on=False))
    self.safety.set_timer(tolerance_us // 2)
    self._rx(self._pcm_cruise_2_msg(main_on=True))
    self.safety.set_timer(tolerance_us + 10)
    self._rx(self._pcm_cruise_2_msg(main_on=False))
    self.safety.set_timer(tolerance_us * 2)
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def _eps_status_msg(self, lka_state: int):
    return self.packer.make_can_msg_safety("EPS_STATUS", 0, {"LKA_STATE": lka_state})

  def _lkas_hud_msg(self, barriers: int):
    return self.packer.make_can_msg_safety("LKAS_HUD", 0, {"BARRIERS": barriers})

  def test_lkas_hud_assist_needs_authority(self):
    # LionDriver DFA DFI-19 / DM-11: in the reference build, an LKAS_HUD that shows assist (BARRIERS != 0) is refused
    # unless authority is granted, or was granted within the last second (the host clears it a few frames later)
    if not LIONDRIVER_REFERENCE:
      raise unittest.SkipTest("reference build only")
    grace_us = 1000000

    # never granted since the safety mode was set
    self._reset_safety_hooks()
    self.safety.set_timer(5000000)
    self.assertTrue(self._tx(self._lkas_hud_msg(0)))
    for barriers in (1, 2, 3):
      self.assertFalse(self._tx(self._lkas_hud_msg(barriers)), barriers)

    # granted: accepted, also before an RX frame has stamped the time
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lkas_hud_msg(1)))
    t0 = 6000000
    self.safety.set_timer(t0)
    self.assertTrue(self._rx(self._speed_msg(0)))

    # withdrawn: accepted within the grace time after the last RX frame with authority, then refused
    self.safety.set_controls_allowed(False)
    self.safety.set_timer(t0 + grace_us)
    self.assertTrue(self._tx(self._lkas_hud_msg(1)))
    self.safety.set_timer(t0 + grace_us + 1)
    self.assertFalse(self._tx(self._lkas_hud_msg(1)))
    self.assertTrue(self._tx(self._lkas_hud_msg(0)))

    # RX frames without authority do not extend the grace time
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertFalse(self._tx(self._lkas_hud_msg(1)))

    # a safety mode change forgets the last grant, also within the grace time
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._speed_msg(0)))
    self._reset_safety_hooks()
    self.assertFalse(self._tx(self._lkas_hud_msg(1)))
    self.safety.set_timer(grace_us // 2)
    self.assertFalse(self._tx(self._lkas_hud_msg(1)))

  def test_eps_fault_blocks_lka(self):
    # LionDriver TSR-110: in the reference build, while EPS_STATUS.LKA_STATE is in the host's steering fault set,
    # STEERING_LKA frames with torque or a steer request are refused; zero frames still pass
    lka_reference = LIONDRIVER_REFERENCE and not self.SAFETY_PARAM & (ToyotaSafetyFlags.SECOC | ToyotaSafetyFlags.ALT_BRAKE |
                                                                        ToyotaSafetyFlags.LTA)
    if not lka_reference:
      raise unittest.SkipTest("reference build, LKA torque control only")
    fault_states = {0, 3, 9, 11, 17, 21, 25}
    for lka_state in range(128):
      with self.subTest(lka_state=lka_state):
        self._reset_safety_hooks()
        self.assertTrue(self._rx(self._eps_status_msg(lka_state)))
        self.safety.set_controls_allowed(True)
        fault = lka_state in fault_states
        self.assertEqual(fault, self.safety.get_safety_eps_fault())  # reported to the host via the panda health
        self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
        self.assertEqual(not fault, self._tx(self._torque_cmd_msg(0, steer_req=1)))
        self.assertEqual(not fault, self._tx(self._torque_cmd_msg(10, steer_req=1)))

    # a safety mode change clears the reported state
    self.assertTrue(self._rx(self._eps_status_msg(25)))
    self.assertTrue(self.safety.get_safety_eps_fault())
    self._reset_safety_hooks()
    self.assertFalse(self.safety.get_safety_eps_fault())

    # the fault clears with the next frame that reports a non-fault state
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._eps_status_msg(25)))
    self.assertFalse(self._tx(self._torque_cmd_msg(10, steer_req=1)))
    self.assertTrue(self._rx(self._eps_status_msg(5)))
    self.assertTrue(self._tx(self._torque_cmd_msg(10, steer_req=1)))

    # a refused frame does not update the torque rate limit state: after the fault, the ramp starts from the last
    # frame actually accepted
    self.assertTrue(self._rx(self._eps_status_msg(9)))
    self.assertFalse(self._tx(self._torque_cmd_msg(25, steer_req=1)))
    self.assertTrue(self._rx(self._eps_status_msg(5)))
    self.assertFalse(self._tx(self._torque_cmd_msg(40, steer_req=1)))  # 25 + 15 would pass, 10 + 30 does not

  def test_reference_rx_checks(self):
    # the reference build supervises EPS_STATUS and PCM_CRUISE_2: a bad checksum is rejected, and without the message
    # a PCM rising edge does not engage
    for msg, _ in self._reference_rx_check_msgs():
      with self.subTest(addr=hex(msg[0].addr)):
        self.assertTrue(self._rx(msg))
        msg[0].data[0] ^= 0x01
        self.assertFalse(self._rx(msg))

        self._reset_safety_hooks()
        self.safety.set_timer(0)
        for m in (self._speed_msg(0), self._user_brake_msg(False), self._user_gas_msg(False), self._torque_meas_msg(0)):
          self._rx(m)
        for m, _ in self._reference_rx_check_msgs():
          if m[0].addr != msg[0].addr:
            self._rx(m)
        self._rx(self._pcm_status_msg(False))
        self._rx(self._pcm_status_msg(True))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_timeout_5_periods(self):
    # LionDriver TSR-401: an RX check lags after 5 nominal periods, without the default 1 s floor.
    # WHEEL_SPEEDS (0xAA, 83 Hz) has the shortest period, so it is the first to time out.
    threshold_us = int(1e6 / 83) * 5
    for elapsed_us, valid in ((threshold_us, True), (threshold_us + 1, False)):
      with self.subTest(elapsed_us=elapsed_us):
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self._rx_all_checked_msgs()
        self.safety.safety_tick()
        self.assertTrue(self.safety.safety_config_valid())

        self.safety.set_timer(elapsed_us)
        self.safety.set_controls_allowed(True)
        self.safety.safety_tick()
        self.assertEqual(valid, self.safety.safety_config_valid())
        self.assertEqual(valid, self.safety.get_controls_allowed())

  def test_no_engagement_with_invalid_rx_checks(self):
    # LionDriver TSR-306: a PCM rising edge does not grant controls while an RX check is failed,
    # including a timeout that safety_tick has not evaluated yet
    threshold_us = int(1e6 / 83) * 5
    for elapsed_us, engages in ((threshold_us, True), (threshold_us + 1, False)):
      with self.subTest(elapsed_us=elapsed_us):
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self._rx_all_checked_msgs()
        self.safety.set_timer(elapsed_us)
        self._rx(self._pcm_status_msg(False))
        self._rx(self._pcm_status_msg(True))
        self.assertEqual(engages, self.safety.get_controls_allowed())

    # a message never received also blocks engagement
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def _pcm_edge(self, t_us):
    self.safety.set_timer(t_us)
    self._rx_all_checked_msgs()
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))

  def _refresh_rx_cruise_active(self, t_us):
    # every RX-checked message again, with PCM_CRUISE kept active (no new rising edge, gas released)
    self.safety.set_timer(t_us)
    pcm_addr = self._pcm_status_msg(True)[0].addr
    for msg in (self._speed_msg(0), self._user_brake_msg(False), self._pcm_status_msg(True), self._user_gas_msg(False),
                self._torque_meas_msg(0), *[m for m, _ in self._reference_rx_check_msgs()]):
      if msg[0].addr == pcm_addr:
        msg = self.packer.make_can_msg_safety("PCM_CRUISE", 0, {"CRUISE_ACTIVE": 1, "GAS_RELEASED": 1})
      self.assertTrue(self._rx(msg))

  def test_pcm_edge_needs_heartbeat_engaged(self):
    # LionDriver (WP-A-05 LS-06): in the reference build a PCM rising edge grants authority only once the heartbeat
    # reports openpilot engaged, within 1 s of the edge; the grant is made in the safety task
    if not LIONDRIVER_REFERENCE:
      raise unittest.SkipTest("reference build only")
    window_us = 1000000
    for wait_us, engages in ((0, True), (window_us // 2, True), (window_us, True), (window_us + 10000, False)):
      with self.subTest(wait_us=wait_us):
        self._reset_safety_hooks()
        self.safety.set_heartbeat_engaged(False)
        self._pcm_edge(0)
        self.assertFalse(self.safety.get_controls_allowed())
        self.safety.safety_tick()
        self.assertFalse(self.safety.get_controls_allowed())
        self._refresh_rx_cruise_active(wait_us)
        self.safety.set_heartbeat_engaged(True)
        self.safety.safety_tick()
        self.assertEqual(engages, self.safety.get_controls_allowed())

    # an expired window grants nothing later; the next rising edge opens a new one
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self._pcm_edge(window_us * 3)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pcm_edge_with_heartbeat_engaged_grants_at_once(self):
    if not LIONDRIVER_REFERENCE:
      raise unittest.SkipTest("reference build only")
    self.safety.set_heartbeat_engaged(True)
    self._pcm_edge(0)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pending_engagement_cancelled(self):
    # cruise off, a brake press or a failed RX check during the window cancels it
    if not LIONDRIVER_REFERENCE:
      raise unittest.SkipTest("reference build only")
    cancels = {
      "cruise off": lambda: self._rx(self._pcm_status_msg(False)),
      "brake": lambda: self._rx(self._user_brake_msg(True)),
      "rx timeout": lambda: self.safety.set_timer(500000),
    }
    for name, cancel in cancels.items():
      with self.subTest(cancel=name):
        self._reset_safety_hooks()
        self.safety.set_heartbeat_engaged(False)
        self._pcm_edge(0)
        cancel()
        self.safety.safety_tick()
        self.safety.set_heartbeat_engaged(True)
        if name == "rx timeout":
          self.safety.set_timer(500000)
        elif name == "cruise off":
          self.safety.set_timer(100000)
          self._rx_all_checked_msgs()  # cruise stays off
        else:
          self._refresh_rx_cruise_active(100000)
        self.safety.safety_tick()
        self.assertFalse(self.safety.get_controls_allowed())

  def _tx_arrival_msgs(self):
    # LionDriver TSR-408: actuation frames the reference build requires from the SoC while authority is granted, with
    # their timeout in us. LKA torque control without SecOC only; ACC_CONTROL only with openpilot longitudinal
    if not LIONDRIVER_REFERENCE or self.SAFETY_PARAM & (ToyotaSafetyFlags.SECOC | ToyotaSafetyFlags.LTA):
      return []
    msgs = [(self._torque_cmd_msg(0, steer_req=0), 50000)]
    if not self.SAFETY_PARAM & ToyotaSafetyFlags.STOCK_LONGITUDINAL:
      msgs.append((self._accel_msg_343(0), 100000))
    return msgs

  def _rx_only(self, cruise_engaged=True):
    # one cycle of every RX check, with the PCM cruise state given (cruise off would end the engagement)
    for msg, _ in self._rx_check_msgs():
      if msg[0].addr == self._pcm_status_msg(False)[0].addr:
        msg = self._pcm_status_msg(cruise_engaged)
      self.assertTrue(self._rx(msg))

  def _rx_tx_live(self, skip_tx_addr=None):
    # one cycle of a live, engaged bus: every RX check and every supervised actuation frame (except skip_tx_addr)
    self._rx_only()
    for msg, _ in self._tx_arrival_msgs():
      if msg[0].addr != skip_tx_addr:
        self.assertTrue(self._tx(msg))

  def test_tx_arrival_supervision(self):
    # LionDriver TSR-408: while authority is granted, each supervised actuation frame revokes it once it has not
    # arrived for longer than its timeout, while everything else keeps arriving
    msgs = self._tx_arrival_msgs()
    if not msgs:
      raise unittest.SkipTest("reference build, LKA torque control without SecOC")
    for msg, timeout_us in msgs:
      with self.subTest(addr=hex(msg[0].addr)):
        for elapsed_us, allowed in ((timeout_us, True), (timeout_us + 1, False)):
          self._reset_safety_hooks()
          self.safety.set_timer(0)
          self._rx_only(cruise_engaged=False)
          self._rx_tx_live()  # PCM cruise rising edge: authority granted
          self.safety.safety_tick()
          self.assertTrue(self.safety.get_controls_allowed())
          # 10 ms safety task steps up to the check, with the other messages still arriving
          for t in range(10000, elapsed_us, 10000):
            self.safety.set_timer(t)
            self._rx_tx_live(skip_tx_addr=msg[0].addr)
            self.safety.safety_tick()
            self.assertTrue(self.safety.get_controls_allowed(), t)
          self.safety.set_timer(elapsed_us)
          self._rx_tx_live(skip_tx_addr=msg[0].addr)
          self.safety.safety_tick()
          self.assertEqual(allowed, self.safety.get_controls_allowed())

  def test_tx_arrival_starts_at_grant(self):
    # LionDriver TSR-408: frames that stopped before authority was granted get the full timeout from the grant, and
    # are not supervised while authority is not granted
    msgs = self._tx_arrival_msgs()
    if not msgs:
      raise unittest.SkipTest("reference build, LKA torque control without SecOC")
    timeout_us = min(t for _, t in msgs)
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx_only(cruise_engaged=False)
    for msg, _ in msgs:
      self.assertTrue(self._tx(msg))
    self.safety.safety_tick()
    # no actuation frames for 1 s while not engaged: nothing to revoke
    grant_us = 1000000
    for t in range(10000, grant_us + 1, 10000):
      self.safety.set_timer(t)
      self._rx_only(cruise_engaged=False)
      self.safety.safety_tick()
    self._rx_only(cruise_engaged=True)  # PCM cruise rising edge: authority granted
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(grant_us + timeout_us)
    self._rx_only()
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(grant_us + timeout_us + 1)
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_tx_arrival_acc_control_only_with_openpilot_longitudinal(self):
    # LionDriver TSR-408: with stock longitudinal the SoC sends no ACC commands, so a missing ACC_CONTROL is tolerated;
    # STEERING_LKA keeps arriving
    msgs = self._tx_arrival_msgs()
    if not msgs or not self.SAFETY_PARAM & ToyotaSafetyFlags.STOCK_LONGITUDINAL:
      raise unittest.SkipTest("reference build, stock longitudinal")
    self.assertEqual([m[0].addr for m, _ in msgs], [0x2E4])
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx_only(cruise_engaged=False)
    self._rx_tx_live()  # PCM cruise rising edge: authority granted
    for t in range(10000, 300001, 10000):
      self.safety.set_timer(t)
      self._rx_tx_live()
      self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_arrival_not_supervised_outside_reference(self):
    # LionDriver TSR-408 applies to the reference configuration only: elsewhere, missing actuation frames alone do not
    # revoke authority (the RX checks keep arriving)
    if self._tx_arrival_msgs():
      raise unittest.SkipTest("supervised configuration")
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self._rx_only(cruise_engaged=False)
    self._rx_only()  # PCM cruise rising edge: authority granted
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_tick()
    for t in range(10000, 300001, 10000):
      self.safety.set_timer(t)
      self._rx_only()
      self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())

  def _rx_check_msgs(self):
    # one message per RX check of the safety mode, with its nominal frequency in Hz
    return [(self._speed_msg(0), 83), (self._torque_meas_msg(0), 50),
            (self._pcm_status_msg(False), 33), (self._user_brake_msg(False), 40)] + self._reference_rx_check_msgs()

  def test_rx_timeout_5_periods_each_msg(self):
    # LionDriver TSR-401 for every RX check, not only the shortest period: each message
    # times out on its own after 5 of its nominal periods while the others keep arriving
    msgs = self._rx_check_msgs()
    for i, (_, freq) in enumerate(msgs):
      threshold_us = int(1e6 / freq) * 5
      with self.subTest(msg=i, threshold_us=threshold_us):
        self._reset_safety_hooks()
        for t, valid in ((0, True), (threshold_us, True), (threshold_us + 1, False)):
          self.safety.set_timer(t)
          for j, (msg, _) in enumerate(msgs):
            if t == 0 or j != i:
              self.assertTrue(self._rx(msg))
          # the SoC keeps sending its actuation frames (TSR-408), so only the RX timeout is tested
          for msg, _ in self._tx_arrival_msgs():
            self.assertTrue(self._tx(msg))
          self.safety.set_controls_allowed(True)
          self.safety.safety_tick()
          self.assertEqual(valid, self.safety.safety_config_valid())
          self.assertEqual(valid, self.safety.get_controls_allowed())

  def test_diagnostics(self, stock_longitudinal: bool = False, ecu_disabled: bool = True):
    for should_tx, msg in ((False, b"\x6D\x02\x3E\x00\x00\x00\x00\x00"),  # fwdCamera tester present
                           (False, b"\x0F\x03\xAA\xAA\x00\x00\x00\x00"),  # non-tester present
                           (True, b"\x0F\x02\x3E\x00\x00\x00\x00\x00")):
      tester_present = libsafety_py.make_CANPacket(0x750, 0, msg)
      self.assertEqual(should_tx and ecu_disabled and not stock_longitudinal, self._tx(tester_present))

  def test_block_aeb(self, stock_longitudinal: bool = False):
    for controls_allowed in (True, False):
      for bad in (True, False):
        for _ in range(10):
          self.safety.set_controls_allowed(controls_allowed)
          dat = [random.randint(1, 255) for _ in range(7)]
          if not bad:
            dat = [0]*6 + dat[-1:]
          msg = libsafety_py.make_CANPacket(0x283, 0, bytes(dat))
          self.assertEqual(not bad and not stock_longitudinal, self._tx(msg))

  # Only allow LTA msgs with no actuation
  def test_lta_steer_cmd(self):
    for engaged, req, req2, torque_wind_down, angle in itertools.product([True, False],
                                                                  [0, 1], [0, 1],
                                                                  [0, 50, 100],
                                                                  np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0 and torque_wind_down == 0
      self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)),
                       f"{req=} {req2=} {angle=} {torque_wind_down=}")

  def test_rx_hook(self):
    # checksum checks
    for msg in ["trq", "pcm"]:
      self.safety.set_controls_allowed(1)
      if msg == "trq":
        msg = self._torque_meas_msg(0)
      if msg == "pcm":
        msg = self._pcm_status_msg(True)
      self.assertTrue(self._rx(msg))
      msg[0].data[4] = 0
      msg[0].data[5] = 0
      msg[0].data[6] = 0
      msg[0].data[7] = 0
      self.assertFalse(self._rx(msg))
      self.assertFalse(self.safety.get_controls_allowed())

    # quality flag tests
    msg = self._speed_msg(0)
    self.assertTrue(self._rx(msg))

    msg = self._speed_msg(0, quality_flag=False)
    self.assertFalse(self._rx(msg))

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 72.22_ is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 259 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)


class TestToyotaSafetyTorque(TestToyotaSafetyBase, common.MotorTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1500]
  MAX_RT_DELTA = 450
  MAX_TORQUE_ERROR = 350
  TORQUE_MEAS_TOLERANCE = 1  # toyota safety adds one to be conservative for rounding

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 17
  MAX_INVALID_STEERING_FRAMES = 1

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE


class TestToyotaSafetyAngle(TestToyotaSafetyBase, common.AngleSteeringSafetyTest):

  # Angle control limits
  STEER_ANGLE_MAX = 94.9461  # deg
  DEG_TO_CAN = 17.452007  # 1 / 0.0573 deg to can

  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.3, 0.15, 0.15]  # windup limit
  ANGLE_RATE_DOWN = [0.36, 0.26, 0.26]  # unwind limit

  MAX_LTA_ANGLE = 94.9461  # PCS faults if commanding above this, deg
  MAX_MEAS_TORQUE = 1500  # max allowed measured EPS torque before wind down
  MAX_LTA_DRIVER_TORQUE = 150  # max allowed driver torque before wind down

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.LTA

  # Only allow LKA msgs with no actuation
  def test_lka_steer_cmd(self):
    for engaged, steer_req, torque in itertools.product([True, False],
                                                        [0, 1],
                                                        np.linspace(-1500, 1500, 7)):
      self.safety.set_controls_allowed(engaged)
      torque = int(torque)
      self.safety.set_rt_torque_last(torque)
      self.safety.set_torque_meas(torque, torque)
      self.safety.set_desired_torque_last(torque)

      should_tx = not steer_req and torque == 0
      self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(torque, steer_req)))

  def test_lta_steer_cmd(self):
    """
    Tests the LTA steering command message
    controls_allowed:
    * STEER_REQUEST and STEER_REQUEST_2 do not mismatch
    * TORQUE_WIND_DOWN is only set to 0 or 100 when STEER_REQUEST and STEER_REQUEST_2 are both 1
    * Full torque messages are blocked if either EPS torque or driver torque is above the threshold

    not controls_allowed:
    * STEER_REQUEST, STEER_REQUEST_2, and TORQUE_WIND_DOWN are all 0
    """
    for controls_allowed in (True, False):
      for angle in np.arange(-90, 90, 1):
        self.safety.set_controls_allowed(controls_allowed)
        self._reset_angle_measurement(angle)
        self._set_prev_desired_angle(angle)

        self.assertTrue(self._tx(self._lta_msg(0, 0, angle, 0)))
        if controls_allowed:
          # Test the two steer request bits and TORQUE_WIND_DOWN torque wind down signal
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            mismatch = not (req or req2) and torque_wind_down != 0
            should_tx = req == req2 and (torque_wind_down in (0, 100)) and not mismatch
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

        else:
          # Controls not allowed
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            should_tx = not (req or req2) and torque_wind_down == 0
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

    # Test max EPS torque and driver override thresholds (independent of angle, test a few representative angles)
    for angle in (-89, 0, 89):
      self.safety.set_controls_allowed(True)
      self._reset_angle_measurement(angle)
      self._set_prev_desired_angle(angle)

      cases = itertools.product(
        (0, self.MAX_MEAS_TORQUE - 1, self.MAX_MEAS_TORQUE, self.MAX_MEAS_TORQUE + 1, self.MAX_MEAS_TORQUE * 2),
        (0, self.MAX_LTA_DRIVER_TORQUE - 1, self.MAX_LTA_DRIVER_TORQUE, self.MAX_LTA_DRIVER_TORQUE + 1, self.MAX_LTA_DRIVER_TORQUE * 2)
      )

      for eps_torque, driver_torque in cases:
        for sign in (-1, 1):
          for _ in range(6):
            self._rx(self._torque_meas_msg(sign * eps_torque, sign * driver_torque))

          # Toyota adds 1 to EPS torque since it is rounded after EPS factor
          should_tx = (eps_torque - 1) <= self.MAX_MEAS_TORQUE and driver_torque <= self.MAX_LTA_DRIVER_TORQUE
          self.assertEqual(should_tx, self._tx(self._lta_msg(1, 1, angle, 100)))
          self.assertTrue(self._tx(self._lta_msg(1, 1, angle, 0)))  # should tx if we wind down torque

  def test_angle_measurements(self):
    """
    * Tests angle meas quality flag dictates whether angle measurement is parsed, and if rx is valid
    * Tests rx hook correctly clips the angle measurement, since it is to be compared to LTA cmd when inactive
    """
    for steer_angle_initializing in (True, False):
      for angle in np.arange(0, self.STEER_ANGLE_MAX * 2, 1):
        # If init flag is set, do not rx or parse any angle measurements
        for a in (angle, -angle, 0, 0, 0, 0):
          self.assertEqual(not steer_angle_initializing,
                           self._rx(self._angle_meas_msg(a, steer_angle_initializing)))

        final_angle = 0 if steer_angle_initializing else round(angle * self.DEG_TO_CAN)
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), final_angle)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), 0)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)


class TestToyotaAltBrakeSafety(TestToyotaSafetyTorque):

  DBC = "toyota_new_mc_pt_generated"

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.ALT_BRAKE

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  # No LTA message in the DBC
  def test_lta_steer_cmd(self):
    pass


class TestToyotaStockLongitudinalBase(TestToyotaSafetyBase):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS
  # Base addresses minus ACC_CONTROL (0x343)
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191]}

  LONGITUDINAL = False

  def test_diagnostics(self, stock_longitudinal: bool = True, ecu_disabled: bool = True):
    super().test_diagnostics(stock_longitudinal=stock_longitudinal, ecu_disabled=ecu_disabled)

  def test_block_aeb(self, stock_longitudinal: bool = True):
    super().test_block_aeb(stock_longitudinal=stock_longitudinal)

  def test_acc_cancel(self):
    """
      Regardless of controls allowed, never allow ACC_CONTROL if cancel bit isn't set
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        self.assertFalse(self._tx(self._accel_msg_343(accel)))
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


class TestToyotaStockLongitudinalTorque(TestToyotaStockLongitudinalBase, TestToyotaSafetyTorque):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL


class TestToyotaStockLongitudinalAngle(TestToyotaStockLongitudinalBase, TestToyotaSafetyAngle):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.LTA


class TestToyotaSecOcSafetyBase(TestToyotaSafetyBase):

  DBC = "toyota_secoc_pt_generated"

  TX_MSGS = TOYOTA_SECOC_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131]}

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.SECOC

  def test_diagnostics(self, ecu_disabled: bool = False):
    super().test_diagnostics(ecu_disabled=ecu_disabled)

  # This platform also has alternate brake and PCM messages, but same naming in the DBC, so same packers work

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL_USER": gas}
    return self.packer.make_can_msg_safety("GAS_PEDAL", 0, values)

  def _rx_check_msgs(self):
    return [(self._speed_msg(0), 83), (self._torque_meas_msg(0), 50), (self._pcm_status_msg(False), 32),
            (self._user_gas_msg(0), 42), (self._user_brake_msg(False), 50)]

  # This platform sends both STEERING_LTA (same as other Toyota) and STEERING_LTA_2 (SecOC signed)
  # STEERING_LTA is checked for no-actuation by the base class, STEERING_LTA_2 is checked for no-actuation below

  def _lta_2_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd}
    return self.packer.make_can_msg_safety("STEERING_LTA_2", 0, values)

  def test_lta_2_steer_cmd(self):
    for engaged, req, req2, angle in itertools.product([True, False], [0, 1], [0, 1], np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0
      self.assertEqual(should_tx, self._tx(self._lta_2_msg(req, req2, angle)), f"{req=} {req2=} {angle=}")

  def _accel_msg_183(self, accel):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_safety("ACC_CONTROL_2", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_183(accel)


class TestToyotaSecOcSafetyStockLongitudinal(TestToyotaSecOcSafetyBase, TestToyotaStockLongitudinalBase):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.SECOC


class TestToyotaSecOcSafety(TestToyotaSecOcSafetyBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183]}

  @unittest.skip("test not applicable for cars without a DSU")
  def test_block_aeb(self, stock_longitudinal: bool = False):
    pass

  def test_343_actuation_blocked(self):
    """
    For SecOC cars, longitudinal acceleration must be sent in ACC_CONTROL_2, but all other ACC
    data remains in ACC_CONTROL. Verify no actuation is sent via ACC_CONTROL.
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel)))
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


if __name__ == "__main__":
  unittest.main()
