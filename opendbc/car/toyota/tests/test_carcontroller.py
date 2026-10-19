import unittest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.carcontroller import ACCEL_FINAL_RATE_LIMIT
from opendbc.car.toyota.values import CAR

# LionDriver WP-W-02 OI-15 and TSR-713 (DQ-300 A): the ACCEL_CMD sent to the PCM is rate limited after the PID and
# pitch compensation, and is never positive while the car reports a stock AEB event

REFERENCE = CAR.TOYOTA_COROLLA_TSS2
LongCtrlState = structs.CarControl.Actuators.LongControlState
ENVELOPE_J = 5.0  # m/s^3, provisional j_up/j_down (WP-W-02 §2.5)


class TestToyotaAccelCommand(unittest.TestCase):
  def setUp(self):
    CarInterface = interfaces[REFERENCE]
    self.CP = CarInterface.get_non_essential_params(REFERENCE)
    assert self.CP.openpilotLongitudinalControl
    self.CI = CarInterface(self.CP)
    self.CS = self.CI.CS
    self.CS.out = structs.CarState(vEgo=25., aEgo=0.)

  def run_controller(self, accel, steps, pitch=0., aeb=lambda f: False, a_ego=0.):
    """Steps the controller at 100 Hz and returns (step, ACCEL_CMD) for each 0x343 frame (every third step)."""
    sent = []
    for frame in range(steps):
      self.CS.out.aEgo = a_ego
      self.CS.out.stockAeb = aeb(frame)
      CC = structs.CarControl.new_message(enabled=True, longActive=True,
                                          orientationNED=[0., pitch(frame) if callable(pitch) else pitch, 0.])
      CC.actuators.accel = accel(frame) if callable(accel) else accel
      CC.actuators.longControlState = LongCtrlState.pid
      actuators, _ = self.CI.apply(CC.as_reader(), frame * int(DT_CTRL * 1e9))
      if frame % 3 == 0:
        sent.append((frame, actuators.accel))
    return sent

  def assert_rate_bounded(self, sent, limit):
    for (_, prev), (frame, cur) in zip(sent, sent[1:], strict=False):
      self.assertLessEqual(abs(cur - prev), limit + 1e-6, f"frame {frame}: {prev:.3f} -> {cur:.3f}")

  def test_step_request_rate_bounded(self):
    # a step from -3.5 to +2.0 m/s^2 and back
    sent = self.run_controller(lambda f: 2.0 if 100 <= f < 400 else -3.5, 700)
    self.assert_rate_bounded(sent, ACCEL_FINAL_RATE_LIMIT)

  def test_pitch_change_rate_bounded(self):
    # the pitch compensation passes a high-pass of the pitch: a sudden 5 % grade change adds a step to the command
    sent = self.run_controller(0.5, 600, pitch=lambda f: 0.05 if f >= 200 else 0.)
    self.assert_rate_bounded(sent, ACCEL_FINAL_RATE_LIMIT)

  def test_below_envelope_jerk_bound(self):
    # one 0x343 frame is three control steps; the raw change per frame stays below the envelope's provisional bound
    self.assertLess(ACCEL_FINAL_RATE_LIMIT, ENVELOPE_J * DT_CTRL * 3)
    self.assertLess(round(ACCEL_FINAL_RATE_LIMIT * 1000), round(ENVELOPE_J * DT_CTRL * 3 * 1000))

  def test_stock_aeb_no_positive_command_from_first_frame(self):
    # the controller has settled at +1.5 m/s^2, with the PID winding up against an ego acceleration of 0
    sent = self.run_controller(1.5, 600, aeb=lambda f: f >= 300)
    self.assertTrue(all(a > 0. for f, a in sent if 200 <= f < 300))
    self.assertTrue(all(a <= 0. for f, a in sent if f >= 300))

  def test_stock_aeb_keeps_braking_request(self):
    sent = self.run_controller(-2.0, 600, aeb=lambda f: f >= 300, a_ego=-2.0)
    self.assertTrue(all(a < 0. for f, a in sent if f >= 300))

  def test_after_stock_aeb_recovers_at_the_rate_limit(self):
    # event from step 200 to 300, then normal control again: the command rises from 0 at no more than the limit
    sent = self.run_controller(1.5, 600, aeb=lambda f: 200 <= f < 300)
    after = [(f, a) for f, a in sent if f >= 297]
    self.assertLessEqual(after[0][1], 0.)
    self.assert_rate_bounded(after, ACCEL_FINAL_RATE_LIMIT)
    self.assertGreater(sent[-1][1], 0.)


if __name__ == "__main__":
  unittest.main()
