import unittest
from types import SimpleNamespace

from opendbc.car.can_definitions import CanData
from opendbc.car.honda.eps48a2 import (Eps48A2Client, FLOW_CONTROL, FRAME_TIMEOUT_NS, OBD_BUS, PREFIX, REQUEST,
                                      RESPONSE_TIMEOUT_NS, RX_ADDR, TX_ADDR, b510_layout_verified)
from opendbc.car.honda.values import CAR
from openpilot.cereal import log
from openpilot.selfdrive.car.eps48a2 import startup_probe
from openpilot.tools.debug.eps48a2 import records


def response():
  data = bytearray(56)
  for offset, value in ((0, -4096), (3, -400), (5, -12), (7, -808)):
    data[offset:offset + 2] = value.to_bytes(2, "big", signed=True)
  data[2], data[9] = 128, 3
  return PREFIX + data


def frames(payload=None):
  payload = response() if payload is None else payload
  result = [bytes([0x10, len(payload)]) + payload[:6]]
  result += [(bytes([0x20 | (i + 1)]) + payload[6 + i * 7:13 + i * 7]).ljust(8, b"\x00")
             for i in range((len(payload) - 6 + 6) // 7)]
  return result


def packet(data, bus=OBD_BUS, address=RX_ADDR):
  return [[CanData(address, data, bus)]]


class TestEps48A2(unittest.TestCase):
  def setUp(self):
    self.client = Eps48A2Client(OBD_BUS, "startup", True)
    self.now = 100_000_000

  def start(self, client=None):
    client = self.client if client is None else client
    self.assertEqual(client.update([], self.now), [CanData(TX_ADDR, REQUEST, OBD_BUS)])
    self.assertFalse(client.result.sampleValid)

  def finish(self, client=None):
    client = self.client if client is None else client
    for i, frame in enumerate(frames()):
      outgoing = client.update(packet(frame), self.now + (i + 1) * 10_000_000)
      self.assertEqual(outgoing, [CanData(TX_ADDR, FLOW_CONTROL, OBD_BUS)] if i == 0 else [])

  def test_multiframe_decoding_and_single_query(self):
    self.start()
    self.client.update(packet(REQUEST, OBD_BUS + 128, TX_ADDR), self.now + 1)
    self.finish()
    result = self.client.result
    self.assertEqual((result.status, result.request, result.requestFlags), ("ok", -4096, 128))
    self.assertEqual((result.directional, result.returnSdiv32, result.q, result.internalStatus), (-400, -12, -808, 3))
    self.assertTrue(result.sampleValid)
    self.assertTrue(result.requestTxSeen)
    self.assertEqual(result.response, response())
    self.assertEqual(result.responseFrames, 9)
    self.assertEqual(self.client.update([], self.now + 10_000_000_000), [])

  def test_raw_only_for_unknown_layout(self):
    client = Eps48A2Client(OBD_BUS, "startup")
    self.start(client)
    self.finish(client)
    self.assertEqual(client.result.status, "ok")
    self.assertEqual(client.result.response, response())
    self.assertFalse(client.result.sampleValid)

  def test_negative_response(self):
    self.start()
    self.client.update(packet(bytes.fromhex("037f223100000000")), self.now + 1)
    self.assertEqual(self.client.result.status, "negativeResponse")
    self.assertEqual(self.client.result.negativeResponseCode, 0x31)
    self.assertFalse(self.client.result.sampleValid)

  def test_pending_has_a_hard_deadline(self):
    self.start()
    for i in range(1, 8):
      self.client.update(packet(bytes.fromhex("037f227800000000")), self.now + i * 250_000_000)
      self.assertTrue(self.client.active)
    self.client.update([], self.now + RESPONSE_TIMEOUT_NS)
    self.assertEqual(self.client.result.status, "timeout")

  def test_timeout_does_not_accept_late_first_frame(self):
    self.start()
    self.assertEqual(self.client.update(packet(frames()[0]), self.now + FRAME_TIMEOUT_NS), [])
    self.assertEqual(self.client.result.status, "timeout")

  def test_partial_response_retained_on_timeout(self):
    for pending in (False, True):
      self.client = Eps48A2Client(OBD_BUS, "startup", True)
      self.start()
      if pending:
        self.client.update(packet(bytes.fromhex("037f227800000000")), self.now + 1)
      self.client.update(packet(frames()[0]), self.now + 2)
      self.client.update([], self.now + FRAME_TIMEOUT_NS + 2)
      self.assertEqual(self.client.result.response, response()[:6])
      self.assertFalse(self.client.result.sampleValid)

  def test_wrong_bus_address_and_unsolicited_response(self):
    self.assertEqual(self.client.update(packet(frames()[0]), self.now), [CanData(TX_ADDR, REQUEST, OBD_BUS)])
    for bus, address in ((0, RX_ADDR), (2, RX_ADDR), (OBD_BUS + 128, RX_ADDR), (OBD_BUS, RX_ADDR + 1)):
      self.assertEqual(self.client.update(packet(frames()[0], bus, address), self.now + 1), [])
    self.assertEqual(self.client.result.responseFrames, 0)

  def test_malformed_frames_fail_closed(self):
    bad_frames = [b"", b"\x10", bytes.fromhex("10ff6248a2000000"),
                  bytes.fromhex("103b6248a3000000"), bytes.fromhex("2100000000000000"),
                  bytes.fromhex("3000000000000000"), bytes.fromhex("086248a200000000")]
    for data in bad_frames:
      with self.subTest(data=data):
        client = Eps48A2Client(OBD_BUS, "startup", True)
        self.start(client)
        self.assertEqual(client.update(packet(data), self.now + 1), [])
        self.assertEqual(client.result.status, "protocolError")
        self.assertFalse(client.result.sampleValid)

  def test_wrong_sequence_and_duplicate_first_frame(self):
    for bad in (frames()[2], frames()[0]):
      client = Eps48A2Client(OBD_BUS, "startup", True)
      self.start(client)
      outgoing = client.update([[CanData(RX_ADDR, f, OBD_BUS) for f in (frames()[0], bad)]], self.now + 1)
      self.assertEqual(outgoing, [])
      self.assertEqual(client.result.status, "protocolError")

  def test_bad_did_and_short_positive_response(self):
    self.start()
    self.client.update(packet(bytes.fromhex("036248a200000000")), self.now + 1)
    self.assertEqual(self.client.result.status, "protocolError")

  def test_runtime_backoff_and_failure_limit(self):
    client = Eps48A2Client(OBD_BUS, "onroad", True)
    self.start(client)
    for i in range(3):
      fail_time = self.now + i * 2_500_000_000 + FRAME_TIMEOUT_NS
      self.assertEqual(client.update([], fail_time), [])
      self.assertEqual(client.result.status, "timeout")
      self.assertEqual(client.update([], fail_time + RESPONSE_TIMEOUT_NS - 1), [])
      outgoing = client.update([], fail_time + RESPONSE_TIMEOUT_NS)
      self.assertEqual(len(outgoing), 1 if i < 2 else 0)
    self.assertTrue(client.completed)

  def test_gate_pauses_without_sending(self):
    client = Eps48A2Client(OBD_BUS, "onroad")
    self.assertEqual(client.update([], self.now, allow_request=False), [])
    self.start(client)
    self.assertEqual(client.update(packet(frames()[0]), self.now + 1, allow_request=False), [])
    self.assertEqual(client.result.status, "transportError")

  def test_layout_check(self):
    CP = SimpleNamespace(carFw=[SimpleNamespace(ecu="eps", address=TX_ADDR, fwVersion=b"39990-T14-B510\x00\x00")])
    self.assertTrue(b510_layout_verified(CP))
    CP.carFw[0].fwVersion = b"39990-T14-B030\x00\x00"
    self.assertFalse(b510_layout_verified(CP))

  def test_only_supported_transport(self):
    for bus, mode in ((0, "startup"), (2, "onroad"), (OBD_BUS, "anything")):
      with self.assertRaises(ValueError):
        Eps48A2Client(bus, mode)

  def test_result_survives_rlog_roundtrip_and_late_logger(self):
    self.start()
    self.finish()
    # Exactly the persistent field used by card.state_publish, after the probe
    # and again in a later segment. No dependence on early CAN subscribers.
    for timestamp in (10_000_000_000, 70_000_000_000):
      event = log.Event.new_message(logMonoTime=timestamp, valid=True)
      event.init("carStateSP").eps48a2 = self.client.result.to_dict()
      with log.Event.from_bytes(event.to_bytes()) as decoded:
        result = decoded.carStateSP.eps48a2
        self.assertEqual(result.status, "ok")
        self.assertEqual(result.q, -808)
        self.assertEqual(result.response, response())
        self.assertEqual(result.requestMonoTime, self.now)
        self.assertEqual(result.bus, OBD_BUS)

  def test_failure_is_explicit_in_rlog(self):
    self.start()
    self.client.update([], self.now + FRAME_TIMEOUT_NS)
    event = log.Event.new_message(valid=True)
    event.init("carStateSP").eps48a2 = self.client.result.to_dict()
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      self.assertEqual(decoded.carStateSP.eps48a2.status, "timeout")
      self.assertFalse(decoded.carStateSP.eps48a2.sampleValid)

  def test_extractor_deduplicates_and_marks_invalid_fields(self):
    self.start()
    self.client.update([], self.now + FRAME_TIMEOUT_NS)
    event = log.Event.new_message(valid=True, logMonoTime=self.now)
    event.init("carStateSP").eps48a2 = self.client.result.to_dict()
    decoded = list(records([event, event]))
    self.assertEqual(len(decoded), 1)
    self.assertEqual(decoded[0]["status"], "timeout")
    self.assertIsNone(decoded[0]["q"])
    self.assertIsNone(decoded[0]["returnSdiv32"])

  def test_pending_then_success_clears_nrc(self):
    self.start()
    self.client.update(packet(bytes.fromhex("037f227800000000")), self.now + 1)
    self.finish()
    self.assertEqual(self.client.result.status, "ok")
    self.assertEqual(self.client.result.negativeResponseCode, 0)

  def test_startup_probe_success_failure_and_skip(self):
    CP = SimpleNamespace(carFingerprint=CAR.HONDA_CITY_7G, carFw=[])
    now = [self.now]
    incoming, sent, switches = [], [], []
    obd_enabled = [False]

    def set_obd(enabled):
      switches.append(enabled)
      obd_enabled[0] = enabled

    def receive():
      result = incoming[:]
      incoming.clear()
      return result

    def send(messages):
      self.assertTrue(obd_enabled[0])
      sent.extend(messages)
      for msg in messages:
        self.assertEqual(msg.src, OBD_BUS)
        incoming.append([CanData(RX_ADDR, f, OBD_BUS) for f in (frames()[:1] if msg.dat == REQUEST else frames()[1:])])

    def sleep(seconds):
      now[0] += int(seconds * 1e9)

    result = startup_probe(CP, receive, send, set_obd, clock=lambda: now[0], sleep=sleep)
    self.assertEqual(result.status, "ok")
    self.assertEqual([m.dat for m in sent], [REQUEST, FLOW_CONTROL])
    self.assertEqual(switches, [True, False])
    self.assertFalse(obd_enabled[0])
    self.assertEqual(startup_probe(CP, receive, send, set_obd, replay=True).status, "skipped")
    CP.carFingerprint = "NOT_A_CITY"
    self.assertEqual(startup_probe(CP, receive, send, set_obd).status, "skipped")
    self.assertEqual(switches, [True, False])

    CP.carFingerprint = CAR.HONDA_CITY_7G
    def broken_receive():
      raise OSError("test transport failure")
    switches.clear()
    self.assertEqual(startup_probe(CP, broken_receive, send, set_obd).status, "transportError")
    self.assertEqual(switches, [True, False])
    self.assertFalse(obd_enabled[0])

  def test_startup_obd_route_failure_and_timeout(self):
    CP = SimpleNamespace(carFingerprint=CAR.HONDA_CITY_7G, carFw=[])
    now, switches, sent = [self.now], [], []

    def set_obd(enabled):
      switches.append(enabled)
      if enabled:
        raise OSError("OBD route not acknowledged")

    result = startup_probe(CP, lambda: [], sent.extend, set_obd)
    self.assertEqual(result.status, "transportError")
    self.assertEqual(sent, [])
    self.assertEqual(switches, [True, False])

    def sleep(seconds):
      now[0] += int(seconds * 1e9)

    switches.clear()
    result = startup_probe(CP, lambda: [], sent.extend, switches.append, clock=lambda: now[0], sleep=sleep)
    self.assertEqual(result.status, "timeout")
    self.assertEqual(result.bus, OBD_BUS)
    self.assertEqual(switches, [True, False])
    self.assertEqual([m.src for m in sent], [OBD_BUS])


if __name__ == "__main__":
  unittest.main()
