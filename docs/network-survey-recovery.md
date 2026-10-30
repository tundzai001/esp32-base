# Network and Base recovery

Firmware version remains V2.3.5.

- SIM/GPRS association alone does not prove usable connectivity. If MQTT and NTRIP both remain disconnected for 30 seconds, try saved Wi-Fi credentials and check the configured MQTT destination before switching clients. A connected service prevents fallback. The saved SIM preference is preserved. Only fallback Wi-Fi is eligible for automatic return to SIM: require two successful MQTT handshakes 60 seconds apart while Wi-Fi is connected, or one successful handshake on a 15-second retry interval if Wi-Fi disconnects. Manual Wi-Fi remains selected. Probes have a separate client ID, no LWT, and no publish/subscribe. The modem remains powered for recovery checks; its LED is not an indicator of the active data transport.
- MQTT writes on the modem are limited to 1024-byte chunks. A short write closes the socket; the packet prefix is never replayed on that connection.
- Boot configuration restores a validated saved Fixed LLA, preserves Auto Base, or starts a fresh 120-second survey. Source streaming requires successful Base initialization as well as the existing license, provisioning and station-lock checks.
- The manual Base status command and periodic polling use the same receiver readback. u-blox polls CFG-TMODE3 and NAV-SVIN with checksum validation and bounded timeouts; querying preserves bytes for the existing RTCM/NMEA stream parser.
- NAV-SVIN mean ECEF coordinates are converted to WGS84 during an active survey and reported as `SURVEY-IN-ESTIMATE`. Only `valid && !active` with a valid position produces `FIXEDPOS` and a saved survey result. Observation time alone cannot complete a survey.

## Validation

```sh
pio run -e tdm240x_dev_general
c++ -std=c++17 -fsanitize=address,undefined -g -Itest/host -Iinclude \
  test/host/base_checks.cpp src/functions/Unicore_Log.cpp src/functions/gnss_cmd_builder.cpp \
  src/functions/ublox_commands.cpp src/functions/unicore_commands.cpp \
  -o /tmp/base-checks
/tmp/base-checks
c++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
  test/host/mqtt_write_checks.cpp -o /tmp/mqtt-write-checks
/tmp/mqtt-write-checks
```

Host checks mock hardware I/O. They do not prove modem handover or GNSS accuracy on a physical device. Dashboard rendering is maintained separately from this firmware repository.
