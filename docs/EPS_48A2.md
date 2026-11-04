# Honda City EPS 48A2 startup capture

This branch makes one read-only `22 48 A2` request after vehicle identification and before `FirmwareQueryDone` / `ControlsReady`. It uses the normal `card → sendcan → pandad → Panda` path while fingerprinting safety is still selected. It does not change the EPS firmware, UDS session, security access, actuation commands, Panda safety rules or OBD routing.

The probe runs only for `HONDA_CITY_7G`, on its directly connected powertrain bus 0. The physical request address `0x18DA30F1` is the EPS address in the Honda firmware database; the response address is `0x18DAF130`. The probe has a 500 ms inter-frame timeout and an absolute two-second response deadline, including response-pending replies. Identification continues if the ECU rejects or ignores the request.

## Find the result in a full rlog

`carStateSP.eps48a2` contains the result on **every subsequent car-state publication**, including later segments. Therefore a logger connecting after the startup query still records its result. The CAN request and response frames also use the existing logged `sendcan` / `can` services, subject to when the logger connected.

The diagnostic record includes:

- `mode`, `status`, `sequence`, request/first-response/completion monotonic timestamps;
- bus, physical addresses, full or partial raw UDS response, NRC and failure detail;
- observed request/flow-control TX echoes (absence of an echo does not independently prove a blocked transmission);
- `sampleValid` and `layoutVerified`, separate from transport success;
- received request, flags, directional code, **S divided by 32**, **q**, and internal status when the B510 layout is verified.

Success, negative response, timeout, malformed ISO-TP and transport exceptions are explicit statuses. A zero value with `sampleValid=false` is not a measurement. `S/32` loses five bits of resolution; multiplying it by 32 does not recover the original S. S and q are software codes, not physical torque, and the ECU reads these fields sequentially.

The B510 layout is verified only when the EPS firmware identity in `CarParams.carFw` contains `T14-B510` or `T14B510`. Other identities, including B030 or absent firmware entries, retain the raw response without interpreting it as B510 S/q. A cached identity is provenance, not proof of the current flash contents.

To extract JSON lines from one or more complete rlogs in the branch's Python environment:

```bash
python -m openpilot.tools.debug.eps48a2 /path/to/rlog.zst
```

The extractor removes repeated identical records and emits null for unverified numeric fields. It preserves the original request/response timestamps, so the repeated startup result is not mistaken for a new sample.

## Validation and scope

```bash
PYTHONPATH="$PWD:$PWD/opendbc_repo" python -m unittest openpilot.selfdrive.car.tests.test_eps48a2
```

Tests cover multi-frame transport, malformed and unrelated frames, timeouts, bounded response-pending, explicit failure records, and actual Cap'n Proto rlog-event serialization, including a logger starting after the probe and a later segment. The probe never runs in replay. A diagnostic response may still be unavailable in the physical vehicle; that is a result to record, not an assumed zero sample.

This branch only validates startup access. It does not enable diagnostic polling under Honda control safety or establish a sampling rate sufficient to resolve steering vibration. Validate a real capture with the vehicle stationary before interpreting it. No ECU query or device installation was performed while developing this change.
