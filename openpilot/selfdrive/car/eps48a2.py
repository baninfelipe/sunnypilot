"""Honda City EPS capture adapters. Startup probing finishes before ControlsReady."""
import time

from opendbc.car.honda.eps48a2 import Eps48A2Client, Eps48A2Result, OBD_BUS, b510_layout_verified
from opendbc.car.honda.values import CAR, HondaFlags, HondaSafetyFlags
from opendbc.car import structs


def configure_onroad_capture(CP, replay: bool = False):
  if (replay or CP.passive or CP.carFingerprint != CAR.HONDA_CITY_7G or
      not CP.flags & HondaFlags.BOSCH_RADARLESS or CP.flags & HondaFlags.BOSCH_CANFD or
      len(CP.safetyConfigs) != 1 or CP.safetyConfigs[0].safetyModel != structs.CarParams.SafetyModel.hondaBosch):
    return None
  CP.safetyConfigs[0].safetyParam |= int(HondaSafetyFlags.EPS_48A2)
  return Eps48A2Client(OBD_BUS, "onroad", b510_layout_verified(CP))


def onroad_capture_ready(CP, panda_states) -> bool:
  return (len(CP.safetyConfigs) == 1 and len(panda_states) == 1 and panda_states[0].safetyModel == structs.CarParams.SafetyModel.hondaBosch and
          panda_states[0].safetyParam == CP.safetyConfigs[0].safetyParam and
          bool(panda_states[0].safetyParam & HondaSafetyFlags.EPS_48A2))


def startup_probe(CP, can_recv, can_send, set_obd_multiplexing, replay: bool = False,
                  clock=time.monotonic_ns, sleep=time.sleep) -> Eps48A2Result:
  if CP.carFingerprint != CAR.HONDA_CITY_7G or replay:
    return Eps48A2Result(status="skipped", detail="unsupported platform or replay")
  client = Eps48A2Client(OBD_BUS, "startup", b510_layout_verified(CP))
  try:
    try:
      # get_car restores normal routing. Re-enable the same acknowledged OBD
      # route used by Honda firmware queries before sending anything on bus 1.
      set_obd_multiplexing(True)
      can_recv()
      started = clock()
      while not client.completed and clock() - started <= 2_100_000_000:
        outgoing = client.update(can_recv(), clock())
        if outgoing:
          can_send(outgoing)
        if not client.completed:
          sleep(0.005)
      if not client.completed:
        client.pause(clock(), "startup capture budget exhausted")
    finally:
      # Restore fingerprinting's normal route, including on send/receive errors.
      set_obd_multiplexing(False)
  except Exception as exc:
    client.result.status = "transportError"
    client.result.sampleValid = False
    client.result.detail = f"{type(exc).__name__}: {exc}"
    client.result.responseMonoTime = clock()
  return client.result
