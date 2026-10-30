import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, CarControllerParams, HondaFlags


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


class TestHondaCitySteerSpeed(unittest.TestCase):
  def test_city_eps_firmware_steer_speed(self):
    cases = (
      ([], 23.),
      ([structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B510\x00\x00")], 20.),
      ([structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B510")], 20.),
      ([structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B030\x00\x00")], 23.),
      ([structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B511\x00\x00")], 23.),
      ([structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B510,modified")], 23.),
      ([structs.CarParams.CarFw.new_message(ecu="fwdRadar", fwVersion=b"39990-T14-B510\x00\x00")], 23.),
    )
    for car_fw, speed in cases:
      for alpha_long in (False, True):
        with self.subTest(car_fw=car_fw, alpha_long=alpha_long):
          CP = CarInterface.get_params(CAR.HONDA_CITY_7G, gen_empty_fingerprint(), car_fw, alpha_long, False, False)
          self.assertAlmostEqual(CP.minSteerSpeed, speed * CV.KPH_TO_MS, places=5)
          self.assertEqual(CarControllerParams(CP).STEER_MAX, 4096)
          self.assertEqual(CP.lateralTuning.which(), "torque")
          self.assertAlmostEqual(CP.lateralTuning.torque.latAccelFactor, 2.)
          self.assertAlmostEqual(CP.lateralTuning.torque.friction, 0.17, places=5)

  def test_other_honda_steer_speed_unchanged(self):
    car_fw = [structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B510\x00\x00")]
    CP = CarInterface.get_params(CAR.HONDA_HRV_3G, gen_empty_fingerprint(), car_fw, False, False, False)
    self.assertAlmostEqual(CP.minSteerSpeed, CAR.HONDA_HRV_3G.config.specs.minSteerSpeed, places=5)

  def test_modified_eps_stays_dashcam_only(self):
    car_fw = [structs.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"39990-T14-B510,modified")]
    CP = CarInterface.get_params(CAR.HONDA_CITY_7G, gen_empty_fingerprint(), car_fw, False, False, False)
    self.assertTrue(CP.dashcamOnly)
