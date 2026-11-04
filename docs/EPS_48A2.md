# Honda City EPS 48A2 startup and onroad capture

The startup probe makes one read-only `22 48 A2` request after vehicle identification and before `FirmwareQueryDone` / `ControlsReady`. It uses the normal `card → sendcan → pandad → Panda` path while fingerprinting safety is still selected. The startup probe does not change the EPS firmware, UDS session, security access, actuation commands, Panda safety rules or OBD routing. The onroad extension below adds a restricted Panda safety permission.

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

The startup probe validates access during identification. Validate a real capture with the vehicle stationary before interpreting it. No ECU query or device installation was performed while developing this change.

## Onroad extension in this branch

This branch also captures the DID during normal Honda City control. It includes the startup probe above, then requests the additional Honda safety parameter `EPS_48A2=32` only for a non-passive, single-Panda Honda City radarless configuration. The collector waits until fresh `pandaStates` confirms the exact requested Honda safety model and parameter and CAN is valid. Replay, passive mode, other platforms, CAN FD and mismatched safety state do not transmit diagnostics.

The client consumes copies of the CAN packets already received by `card`, and appends request/flow-control frames to its existing `can_sends`. It has no blocking I/O or sleep in the control cycle. Missing control initialization or invalid CAN pauses an active transaction. Three consecutive diagnostic failures stop polling until process restart. Diagnostic failures do not change control permissions or actuator requests.

The Panda permission is limited to PT bus 0, physical EPS address `0x18DA30F1`, DLC 8, the exact padded `22 48 A2` single-frame request, and one exact flow-control frame within two seconds of a permitted request. Requests are limited to two per second. The host verifies the response address, bus, first-frame DID/length and consecutive-frame sequence before emitting flow control. Other UDS services, DIDs, buses and payloads remain rejected. No periodic diagnostic response is added to the safety RX watchdogs, so losing diagnostic replies does not create a control-required message.

`mode=onroad` records are retained through the same `carStateSP.eps48a2` field. Deduplicate by request timestamp and sequence; repeated publications are not independent samples. Polling is at most 2 Hz and the 10 ms ISO-TP separation time makes the actual rate lower. **This is transport/state observability, not a sampling rate sufficient to resolve the approximately 40 Hz component previously observed in CAN logs.**

This extension changes Panda safety and must be reviewed under comma's fork policy. It preserves the existing test suite and reruns the complete original stock-longitudinal, alternate-brake and openpilot-longitudinal safety tests with the extra parameter, in addition to new diagnostic rejection/rate/timing tests. It does not constitute comma approval or guarantee server-access eligibility.

## Bundled Panda firmware

This fork ships prebuilt Panda firmware. The onroad branch therefore includes a rebuilt H7 application, not just edits to the safety source. Installing this branch on a device can trigger the existing Panda firmware updater. The bootstub is unchanged; the application retains the fork's existing debug signing convention. No hardware was flashed during development.

The build uses the ARM compiler package already pinned in `uv.lock`, limits the application linker region to `0x08020000..0x08100000`, and checks its reset vector, length and signature. `docs/EPS_48A2_FIRMWARE.json` records source and signed-binary hashes. In a Linux Python 3.12 environment:

```bash
python -m pip install pycryptodome comma-deps-gcc-arm-none-eabi==13.2.1.post103
python panda/scripts/build_eps48a2.py --check
python panda/scripts/build_eps48a2.py --build --check
git diff --exit-code -- panda/board/obj/panda_h7.bin.signed panda/board/obj/panda_h7/main.bin docs/EPS_48A2_FIRMWARE.json
```

CI reproduces the signed application and runs the full safety suite with UBSan and with line coverage. Debug ELF files can differ by build-directory paths; reproducibility is checked against the raw and signed application binaries and the manifest.
