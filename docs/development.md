# Firmware development and upstream review

## Backend agent ownership

`include/functions/Backend_Agent.h` defines the public agent interface. Its
implementation is split by responsibility under `src/functions/`:

| File | Responsibility |
| --- | --- |
| `Backend_Agent.cpp` | Startup, saved state, transport lifecycle and data queues |
| `Backend_Agent_Commands.cpp` | Backend command validation, execution and results |
| `Backend_Agent_Workflows.cpp` | Auto Base, reference checks and GNSS command exchanges |
| `Backend_Agent_Gnss.cpp` | Periodic receiver readback and saved Survey-In results |
| `Backend_Agent_Status.cpp` | Status and health payloads |
| `Backend_Agent_Projection.cpp` | Coordinate transformation and province projections |
| `Backend_Agent_Websocket.cpp` | RFC6455 handshake, framing and TCP reconnects |

`Backend_Agent_Internal.h` and `Backend_Agent_Websocket.h` are implementation
headers. Keep transport framing out of command handling and status formatting.
Survey-In status uses receiver readback; the removed GGA polling state must not
be reintroduced to make RTCM-only mode depend on NMEA.

## Verify a cleanup

Run the build and host checks in [base-4g.md](base-4g.md#kiểm-tra), plus:

```sh
python3 -B test/host/check_gnss_stream.py
python3 -B test/host/check_rover_gga.py
git diff --check
```

These checks cover software behavior. They do not establish antenna position
accuracy or replace verification on the receiver. A structural cleanup should
preserve command bytes, reply handling, NVS keys and MQTT payloads.

## Prepare an upstream change

Fetch the current `long-in-aitogy/esp32-um-ubx-base` `develop` branch and start
the review branch there. The cleanup prepared on 27 October 2026 uses
`6af5a260c9b8f62ad9296d50b01dc66c57a2092e` as its base; fetch again before
publishing because `develop` can advance.

Keep deployment defaults from upstream when transferring firmware from CORS.
Local Wi-Fi credentials and test mountpoints are not firmware fixes. Transfer
intentional command-port routing changes separately from those defaults.
Do not copy `.pio`, `compile_commands.json` or editor databases into the patch.

Review the firmware-only diff against the actual target branch. Separate
mechanical moves from behavior changes when preparing commits. A merge
conflict records overlapping edits; resolve it against the current code and
rerun the checks rather than replacing the target file with an old snapshot.

PR #103 already merged the previous CORS synchronization. PR #66 still points
to an older branch with overlapping changes. The new cleanup is prepared on a
separate branch; changing or closing that PR is a separate repository action.
