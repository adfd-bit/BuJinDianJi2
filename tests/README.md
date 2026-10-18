# Firmware Host Tests

Run the radar parser and obstacle tests with a desktop GCC compiler from the project root:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/hal_stub -ICore/Inc Core/Src/n10_radar.c tests/test_radar.c -lm -o tests/test_radar.exe
.\tests\test_radar.exe
```

The HAL stub supplies only the UART handle type and a controlled clock. Tests feed both legacy 108-byte `A5 5A 6C` packets and LD06 47-byte `54 2C` packets through the same ring buffer as the firmware. LD06 uses little-endian fields, twelve points, inclusive end-angle interpolation, and CRC8 polynomial `0x4D`.

The physical STL-06NBJ also inserts `54 1F 00 00 00 41` every 500 ms. The parser accepts only this six-byte zero-status format with a valid CRC8; nonzero status bodies, unknown headers, and bad CRCs invalidate the scan. The precise status-field semantics are not established by the public SDK. `hardware-rx-09.bin` supplies the real packet for split-packet, continued-scan, expiry, and rejection tests. Status packets do not increase point-cloud frame counts or refresh their timeout.

The hardware replay fixture is `tests/artifacts/hardware-rx.bin`, a 2048-byte receive-ring capture. Its oldest complete packet starts at offset 1929; 42 CRC-valid packets continue through the ring boundary to offset 1808. The replay uses that chronological run twice so a complete scan can form after synchronization, while ignoring the ring's incomplete cut. This verifies decoding of captured hardware bytes, rather than current live hardware operation.

Coverage includes scan synchronization, packets crossing zero degrees, circular obstacle merging, raw-sample centers, nearest-object selection, fractional point gaps and radial distance splits, car-relative coordinates, raw-point front safety, bad checksums and headers, receive loss, UART recovery notification, parser reinitialization, scan expiry and clock wraparound. Published snapshots stay stable while the next scan is assembled.

`hardware-rx-09.bin` also contains the six-byte periodic status packet at offset 1272. Tests verify that its CRC-valid zero status preserves a scan without counting a measurement frame or extending freshness; corrupted CRC, nonzero state and unknown packet types invalidate the scan. The current algorithm suite passes 76 snapshot checks.

UART recovery and TX DMA ownership tests use a separate transport stub:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/transport_stubs -ICore/Inc tests/test_uart_transport.c -o tests/test_uart_transport.exe
.\tests\test_uart_transport.exe
gcc -std=c11 -Wall -Wextra -Werror -Itests/transport_stubs -ICore/Inc tests/test_radar_telemetry.c -o tests/test_radar_telemetry.exe
.\tests\test_radar_telemetry.exe
py -m unittest discover -s tests -p "test_*.py"
```

The UART stub isolates driver state and injects HAL failures. The tests verify USART1 RX / USART2 TX configuration, receive recovery, busy output, in-flight buffer immutability, latest pending snapshots, DMA errors, and bounded JSON formatting. Python tests exercise fragmented JSON, malformed reports, freshness, coordinate projection, and reader-session isolation.

`check_visualizer_gui.py` uses synthetic snapshots for layout checks. `check_visualizer_hardware.py --port COM5` instead receives real reports, requires fresh nonempty obstacles in each view, captures `hardware-visualizer-*.png`, and checks disconnect cleanup. Other `visualizer-*.png` screenshots are offline layout checks and do not represent measurements from the physical radar.

After connecting the physical board, radar, and USB UART (RX -> PA2), close the GUI serial connection and capture real reports:

```powershell
py tests/capture_serial.py --port COM5 --seconds 5 --output tests/artifacts/hardware-serial.jsonl
```

Replace COM5 with the enumerated adapter port. Exit code 0 means valid scans were received, 2 means only invalid status reports were received, and 1 means no valid JSON reports were received. This check does not establish physical target direction or measurement accuracy.
