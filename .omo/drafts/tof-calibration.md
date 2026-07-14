# Draft: ToF Calibration Feature

## Status: awaiting-approval

## Findings

### Architecture
- ATmega328P: Reads AS5600 at 500Hz (2ms), calculates Hz at 10Hz (10ms), sends 22-byte TelemetryPacket via SPI
- ESP32: Transparent SPI→UART bypass at 460800 baud, no processing
- Rust HMI: Receives packets, displays 4 graphs, controls motors

### Current TelemetryPacket (22 bytes)
```
startByte(1) + timestamp_ms(4) + mPWM(2) + sPWM(2) + mESC(2) + sESC(2) + mRPM(2) + sRPM(2) + mHz(2) + sHz(2) + checksum(1) = 22 bytes
```

### User Decisions
1. **Data strategy**: Option A - Add raw delta-angle field to packet (more precise, ~200Hz)
2. **Excitation methods**: Both Sniper and Heartbeat with UI selector
3. **Rope length**: 1-2 meters typical

## Components (Topology Lock)

| ID | Component | Outcome | Status | Evidence |
|----|-----------|---------|--------|----------|
| C1 | ATmega328P Firmware | Add delta-angle fields to TelemetryPacket | Pending | src/main.cpp:89-101 |
| C2 | ESP32 Firmware | Update to forward 26-byte packet | Pending | ESP32 src/main.cpp |
| C3 | Rust Protocol | Update parser for new packet size | Pending | rust_hmi/src/protocol.rs |
| C4 | Rust Serial Comm | Handle new packet format | Pending | rust_hmi/src/serial_comm.rs |
| C5 | Rust HMI UI | Add ToF Calibration tab with 4 cards | Pending | rust_hmi/src/app.rs |
| C6 | Rust HMI State Machine | ToF experiment orchestration | Pending | rust_hmi/src/app.rs |

## Decisions

### Packet Expansion
- Add `mAngle` (int16) and `sAngle` (int16) fields after sHz
- New packet size: 26 bytes
- Delta angle = current_raw - last_raw (wrapped at 4096)

### ToF State Machine (Rust HMI)
States: Idle → Arming → Silence → Firing → Listening → Done
- Arming: Send ARM command, wait 1.5-2s
- Silence: Ignore fluctuations
- Firing: Send pulse (Sniper mode) or mark T0 on relay ON (Heartbeat mode)
- Listening: Monitor sAngle, detect threshold crossing for T1
- Done: Calculate Δt, f1, update UI

### UI Layout
- Tab system: "DAQ Lazo Abierto" (existing) + "Calibración ToF" (new)
- ToF tab has 4 cards as specified in rust-upgrade.txt

## Open Questions
- None remaining - all forks resolved via user answers

## Approach
Modify firmware layer by layer (ATmega → ESP32 → Rust), then add HMI feature.
