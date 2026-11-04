"""Honda City EPS capture adapters. Startup probing finishes before ControlsReady."""
import time

from opendbc.car.honda.eps48a2 import Eps48A2Client, Eps48A2Result, b510_layout_verified
from opendbc.car.honda.values import CAR, HondaFlags, HondaSafetyFlags
from opendbc.car import structs


def configure_onroad_capture(CP, replay: bool = False):
  if (replay or CP.passive or CP.carFingerprint != CAR.HONDA_CITY_7G or
      not CP.flags & HondaFlags.BOSCH_RADARLESS or CP.flags & HondaFlags.BOSCH_CANFD or
      len(CP.safetyConfigs) != 1 or CP.safetyConfigs[0].safetyModel != structs.CarParams.SafetyModel.hondaBosch):
    return None
  CP.safetyConfigs[0].safetyParam |= int(HondaSafetyFlags.EPS_48A2)
  return Eps48A2Client(0, "onroad", b510_layout_verified(CP))


def onroad_capture_ready(CP, panda_states) -> bool:
  return (len(CP.safetyConfigs) == 1 and len(panda_states) == 1 and panda_states[0].safetyModel == structs.CarParams.SafetyModel.hondaBosch and
          panda_states[0].safetyParam == CP.safetyConfigs[0].safetyParam and
          bool(panda_states[0].safetyParam & HondaSafetyFlags.EPS_48A2))


def startup_probe(CP, can_recv, can_send, replay: bool = False,
                  clock=time.monotonic_ns, sleep=time.sleep) -> Eps48A2Result:
  if CP.carFingerprint != CAR.HONDA_CITY_7G or replay:
    return Eps48A2Result(status="skipped", detail="unsupported platform or replay")
  client = Eps48A2Client(0, "startup", b510_layout_verified(CP))
  started = clock()
  try:
    # Fingerprinting leaves OBD multiplexing disabled. Use the directly connected
    # City PT bus, not a cached firmware-query bus that might have used OBD routing.
    can_recv()
    while not client.completed and clock() - started <= 2_100_000_000:
      outgoing = client.update(can_recv(), clock())
      if outgoing:
        can_send(outgoing)
      if not client.completed:
        sleep(0.005)
    if not client.completed:
      client.pause(clock(), "startup capture budget exhausted")
  except Exception as exc:
    client.result.status = "transportError"
    client.result.sampleValid = False
    client.result.detail = f"{type(exc).__name__}: {exc}"
    client.result.responseMonoTime = clock()
  return client.result
