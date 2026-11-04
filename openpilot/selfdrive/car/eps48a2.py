"""Honda City EPS capture adapters. Startup probing finishes before ControlsReady."""
import time

from opendbc.car.honda.eps48a2 import Eps48A2Client, Eps48A2Result, b510_layout_verified
from opendbc.car.honda.values import CAR


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
