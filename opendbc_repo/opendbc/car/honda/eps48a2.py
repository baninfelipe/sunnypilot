"""Bounded, read-only B510 live-data query. No socket, sleep or safety-mode changes."""
from dataclasses import asdict, dataclass

from opendbc.car.can_definitions import CanData

TX_ADDR = 0x18DA30F1
RX_ADDR = 0x18DAF130
REQUEST = bytes.fromhex("032248a200000000")
FLOW_CONTROL = bytes.fromhex("30000a0000000000")
PREFIX = bytes.fromhex("6248a2")
RESPONSE_SIZE = 59  # SID + DID + 56 data bytes
POLL_INTERVAL_NS = 500_000_000
FRAME_TIMEOUT_NS = 500_000_000
RESPONSE_TIMEOUT_NS = 2_000_000_000


@dataclass
class Eps48A2Result:
  enabled: bool = False
  mode: str = "disabled"
  status: str = "disabled"
  sequence: int = 0
  requestMonoTime: int = 0
  firstResponseMonoTime: int = 0
  responseMonoTime: int = 0
  bus: int = 0
  txAddress: int = TX_ADDR
  rxAddress: int = RX_ADDR
  response: bytes = b""
  negativeResponseCode: int = 0
  request: int = 0
  requestFlags: int = 0
  directional: int = 0
  returnSdiv32: int = 0
  q: int = 0
  internalStatus: int = 0
  sampleValid: bool = False
  layoutVerified: bool = False
  detail: str = ""
  responseFrames: int = 0
  requestTxSeen: bool = False
  flowControlTxSeen: bool = False

  def to_dict(self) -> dict:
    return asdict(self)


def b510_layout_verified(CP) -> bool:
  return any(str(fw.ecu) == "eps" and fw.address == TX_ADDR and
             (b"T14-B510" in bytes(fw.fwVersion) or b"T14B510" in bytes(fw.fwVersion)) for fw in CP.carFw)


class Eps48A2Client:
  def __init__(self, bus: int, mode: str, layout_verified: bool = False):
    if mode not in ("startup", "onroad") or bus != 0:
      raise ValueError("48A2 capture is restricted to the City powertrain bus 0")
    self.result = Eps48A2Result(enabled=True, mode=mode, status="waiting", bus=bus, layoutVerified=layout_verified)
    self.active = False
    self.completed = False
    self.failures = 0
    self.next_request_ns = 0
    self._last_progress_ns = 0
    self._payload = bytearray()
    self._next_cf = 1

  def _start(self, now_ns: int) -> CanData:
    previous = self.result
    self.result = Eps48A2Result(enabled=True, mode=previous.mode, status="pending", sequence=previous.sequence + 1,
                               requestMonoTime=now_ns, bus=previous.bus, layoutVerified=previous.layoutVerified)
    self.active = True
    self._last_progress_ns = now_ns
    self._payload.clear()
    self._next_cf = 1
    return CanData(TX_ADDR, REQUEST, self.result.bus)

  def _finish(self, status: str, now_ns: int, detail: str = "") -> None:
    if self._payload and not self.result.response:
      self.result.response = bytes(self._payload)
    self.result.status = status
    self.result.detail = detail
    self.result.responseMonoTime = now_ns
    self.active = False
    self.failures = 0 if status == "ok" else self.failures + 1
    self.completed = self.result.mode == "startup" or self.failures >= 3
    # A quiet period after failure reduces contamination by a late response.
    self.next_request_ns = now_ns + (POLL_INTERVAL_NS if status == "ok" else RESPONSE_TIMEOUT_NS)
    if self.failures >= 3:
      self.result.detail += "; polling stopped after three consecutive failures"

  def pause(self, now_ns: int, reason: str) -> None:
    if self.active:
      self._finish("transportError", now_ns, reason)
    elif self.result.status == "waiting":
      self.result.detail = reason

  def _decode(self, payload: bytes, now_ns: int) -> None:
    self.result.response = payload
    if len(payload) == 3 and payload[:2] == b"\x7f\x22":
      self.result.negativeResponseCode = payload[2]
      if payload[2] == 0x78:
        self._payload.clear()
        self._last_progress_ns = now_ns
        return
      self._finish("negativeResponse", now_ns)
    elif len(payload) != RESPONSE_SIZE or not payload.startswith(PREFIX):
      self._finish("protocolError", now_ns, "unexpected response SID, DID or length")
    else:
      self.result.negativeResponseCode = 0
      if self.result.layoutVerified:
        data = payload[3:]
        def signed(at):
          return int.from_bytes(data[at:at + 2], "big", signed=True)
        self.result.request = signed(0)
        self.result.requestFlags = data[2]
        self.result.directional = signed(3)
        self.result.returnSdiv32 = signed(5)
        self.result.q = signed(7)
        self.result.internalStatus = data[9]
        self.result.sampleValid = True
      self._finish("ok", now_ns, "" if self.result.layoutVerified else "unverified EPS layout; raw response only")

  def update(self, packets: list[list[CanData]], now_ns: int, allow_request: bool = True) -> list[CanData]:
    """Consume copies of received CAN, return at most one request or flow-control frame."""
    outgoing = []
    if not allow_request:
      self.pause(now_ns, "waiting for valid CAN and the configured Honda safety mode")
      return outgoing

    if self.active and (now_ns - self.result.requestMonoTime >= RESPONSE_TIMEOUT_NS or
                        now_ns - self._last_progress_ns >= FRAME_TIMEOUT_NS):
      self._finish("timeout", now_ns, "no complete response; inspect TX echoes and Panda rejection counters")

    for packet in packets:
      for frame in packet:
        if self.active and frame.address == TX_ADDR and frame.src == self.result.bus + 128:
          self.result.requestTxSeen |= bytes(frame.dat) == REQUEST
          self.result.flowControlTxSeen |= bytes(frame.dat) == FLOW_CONTROL
        if not self.active or frame.src != self.result.bus or frame.address != RX_ADDR:
          continue
        data = bytes(frame.dat)
        if len(data) != 8:
          self._finish("protocolError", now_ns, "expected an eight-byte classic CAN frame")
          continue
        if self.result.firstResponseMonoTime == 0:
          self.result.firstResponseMonoTime = now_ns
        self.result.responseFrames += 1
        self._last_progress_ns = now_ns
        kind = data[0] >> 4
        if kind == 0 and not self._payload and 0 < data[0] < 8:
          self._decode(data[1:1 + data[0]], now_ns)
        elif kind == 1 and not self._payload and ((data[0] & 15) << 8 | data[1]) == RESPONSE_SIZE and data[2:5] == PREFIX:
          self._payload.extend(data[2:])
          self._next_cf = 1
          outgoing.append(CanData(TX_ADDR, FLOW_CONTROL, self.result.bus))
        elif kind == 2 and self._payload and data[0] & 15 == self._next_cf:
          self._payload.extend(data[1:1 + min(7, RESPONSE_SIZE - len(self._payload))])
          self._next_cf = (self._next_cf + 1) & 15
          if len(self._payload) == RESPONSE_SIZE:
            self._decode(bytes(self._payload), now_ns)
        else:
          self._finish("protocolError", now_ns, "unexpected ISO-TP frame or sequence")

    if self.active and (now_ns - self.result.requestMonoTime >= RESPONSE_TIMEOUT_NS or
                        now_ns - self._last_progress_ns >= FRAME_TIMEOUT_NS):
      self._finish("timeout", now_ns, "no complete response; inspect TX echoes and Panda rejection counters")
    if not self.active and not self.completed and now_ns >= self.next_request_ns:
      outgoing.append(self._start(now_ns))
    # A malformed follow-up must never leave a flow-control frame queued.
    return outgoing if self.active else []
