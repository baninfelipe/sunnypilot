import unittest

from opendbc.car.honda.eps48a2 import FLOW_CONTROL, OBD_BUS, REQUEST, RX_ADDR, TX_ADDR
from opendbc.car.honda.values import HondaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests import test_honda
from opendbc.safety.tests import common
from opendbc.safety.tests.libsafety import libsafety_py

FIRST_FRAME = bytes.fromhex("103b6248a2000000")


class Eps48A2SafetyTests:
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self._reset_safety_hooks()

  def _reset_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaBosch, self.BASELINE.SAFETY_PARAM | HondaSafetyFlags.EPS_48A2)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def diag(self, data=REQUEST, bus=OBD_BUS, address=TX_ADDR):
    return libsafety_py.make_CANPacket(address, bus, data)

  def response(self, data=FIRST_FRAME, bus=OBD_BUS, address=RX_ADDR):
    return libsafety_py.make_CANPacket(address, bus, data)

  def test_diag_request_rate_and_control_independence(self):
    for enabled in (False, True):
      self._reset_safety_hooks()
      self.safety.set_controls_allowed(enabled)
      self.safety.set_timer(0)
      self.assertTrue(self._tx(self.diag()))
      self.assertFalse(self._tx(self.diag()))
      self.safety.set_timer(499999)
      self.assertFalse(self._tx(self.diag()))
      self.safety.set_timer(500000)
      self.assertTrue(self._tx(self.diag()))
      self.assertEqual(self.safety.get_controls_allowed(), enabled)

  def test_diag_fc_requires_a_request_and_is_single_use(self):
    self.safety.set_timer(0)
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))
    self.assertTrue(self._tx(self.diag()))
    self._rx(self.response())
    self.assertTrue(self._tx(self.diag(FLOW_CONTROL)))
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))
    self._rx(self.response())
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))

  def test_diag_only_exact_payloads(self):
    self.safety.set_timer(0)
    for i in range(8):
      for value in range(256):
        if value != REQUEST[i]:
          data = bytearray(REQUEST)
          data[i] = value
          self.assertFalse(self._tx(self.diag(data)), (i, value))
    self.assertTrue(self._tx(self.diag()))
    self._rx(self.response())
    for i in range(8):
      for value in range(256):
        if value != FLOW_CONTROL[i]:
          data = bytearray(FLOW_CONTROL)
          data[i] = value
          self.assertFalse(self._tx(self.diag(data)), (i, value))
    self.assertTrue(self._tx(self.diag(FLOW_CONTROL)))

  def test_diag_rejects_other_uds_services_and_dids(self):
    requests = ("032248a300000000", "0322f18900000000", "0210030000000000", "0211020000000000",
                "0227010000000000", "0328030300000000", "042e48a201000000", "043101ff00000000")
    for data in requests:
      self.assertFalse(self._tx(self.diag(bytes.fromhex(data))))

  def test_diag_only_exact_bus_address_and_dlc(self):
    for length in (*range(8), 12, 16, 64):
      self.assertFalse(self._tx(self.diag(REQUEST[:length].ljust(length, b"\x00"))))
    for bus in (0, 2, 3):
      self.assertFalse(self._tx(self.diag(bus=bus)))
    for address in (TX_ADDR + 1, TX_ADDR + 0x100, RX_ADDR):
      self.assertFalse(self._tx(self.diag(address=address)))
    self.assertTrue(self._tx(self.diag()))

  def test_diag_ff_and_fc_deadlines(self):
    for timestamp, allowed in ((2000000, True), (2000001, False)):
      self._reset_safety_hooks()
      self.safety.set_timer(0)
      self.assertTrue(self._tx(self.diag()))
      self.safety.set_timer(timestamp)
      self._rx(self.response())
      self.assertEqual(self._tx(self.diag(FLOW_CONTROL)), allowed)
    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self.diag()))
    self._rx(self.response())
    self.safety.set_timer(2000001)
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))

  def test_diag_timer_wrap(self):
    self.safety.set_timer(0xFFFFFF00)
    self.assertTrue(self._tx(self.diag()))
    self.safety.set_timer((0xFFFFFF00 + 1000) & 0xFFFFFFFF)
    self._rx(self.response())
    self.assertTrue(self._tx(self.diag(FLOW_CONTROL)))
    self.assertFalse(self._tx(self.diag()))
    self.safety.set_timer((0xFFFFFF00 + 500000) & 0xFFFFFFFF)
    self.assertTrue(self._tx(self.diag()))

  def test_diag_no_control_enable_or_steering_bypass(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self.diag()))
    self._rx(self.response())
    self.assertTrue(self._tx(self.diag(FLOW_CONTROL)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_diag_mode_reset_clears_pending(self):
    self.assertTrue(self._tx(self.diag()))
    self._rx(self.response())
    self._reset_safety_hooks()
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))
    self.assertTrue(self._tx(self.diag()))

  def test_diag_absent_from_other_configurations(self):
    for model, param in ((CarParams.SafetyModel.hondaNidec, 32), (CarParams.SafetyModel.hondaBosch, 0),
                         (CarParams.SafetyModel.hondaBosch, 32), (CarParams.SafetyModel.hondaBosch, 8),
                         (CarParams.SafetyModel.hondaBosch, 16 | 32), (CarParams.SafetyModel.hondaBosch, 8 | 16 | 32)):
      self.safety.set_safety_hooks(model, param)
      self.assertFalse(self._tx(self.diag()), (model, param))
      self._rx(self.response())
      self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))

  def test_diag_expiry_on_normal_can(self):
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self.diag()))
    self.safety.set_timer(2000001)
    packer = common.CANPackerSafety("honda_bosch_radarless_generated")
    self._rx(packer.make_can_msg_safety("ENGINE_DATA", 0, {"XMISSION_SPEED": 0}))
    self.safety.set_timer(0)  # expired state must stay cleared, even after timer wrap
    self.assertFalse(self._tx(self.diag(FLOW_CONTROL)))

  def test_original_actuation_suite_with_diagnostic_flag(self):
    # Re-run the original variant's complete tests with the extra flag. Keep the
    # original class identity: diagnostic logging is not a different vehicle
    # model with a different set of actuator addresses.
    for name in unittest.defaultTestLoader.getTestCaseNames(self.BASELINE):
      case = self.BASELINE(name)
      case.SAFETY_PARAM = self.BASELINE.SAFETY_PARAM | HondaSafetyFlags.EPS_48A2
      case.TX_MSGS = self.BASELINE.TX_MSGS + [[TX_ADDR, OBD_BUS]]
      result = unittest.TestResult()
      case.run(result)
      self.assertEqual(result.errors, [], name)
      self.assertEqual(result.failures, [], name)


class TestHondaEps48A2(Eps48A2SafetyTests, unittest.TestCase):
  BASELINE = test_honda.TestHondaBoschRadarlessSafety


class TestHondaEps48A2AltBrake(Eps48A2SafetyTests, unittest.TestCase):
  BASELINE = test_honda.TestHondaBoschRadarlessAltBrakeSafety


class TestHondaEps48A2Long(Eps48A2SafetyTests, unittest.TestCase):
  BASELINE = test_honda.TestHondaBoschRadarlessLongSafety


if __name__ == "__main__":
  unittest.main()
