# tof-calibration - Work Plan

## TL;DR (For humans)

**What you'll get:** Una nueva pestaña "Calibración ToF" en el HMI Rust que orquesta experimentos de Tiempo de Vuelo en la cuerda del robot, midiendo la propagación de onda entre Master y Slave usando los encoders AS5600. Soporta dos métodos de excitación (Sniper y Heartbeat) seleccionables desde la UI.

**Why this approach:** Expandir el paquete de telemetría existente (22→26 bytes) con campos de delta-ángulo raw permite detección precisa de la onda a ~200Hz. La orquestación en Rust (no en microcontroladores) mantiene el ATmega como "títere ciego" simplificando el firmware.

**What it will NOT do:** No modifica la funcionalidad DAQ existente (Tab 1). No agrega WiFi/MQTT. No cambia la tasa de muestreo del encoder existente (500Hz I2C).

**Effort:** Medium (3-4 días de implementación)
**Risk:** Medium - Requiere sincronización de protocolo entre 3 capas (ATmega→ESP32→Rust)
**Decisions to sanity-check:** Tamaño del paquete (26 bytes), tasa de muestreo de ángulo (~200Hz), timeout del experimento (5s max)

Your next move: Aprobar el plan, o solicitar revisión de alta precisión (Momus). El detalle completo de ejecución sigue abajo.

---

> TL;DR (machine): Medium effort, 6-component cross-layer feature: firmware packet expansion + ESP32 passthrough + Rust protocol parser + serial comm + HMI tab UI + ToF state machine.

## Scope
### Must have
- Campo `mAngle` (int16) y `sAngle` (int16) en TelemetryPacket (26 bytes total)
- Delta-ángulo calculado en ATmega328P a ~200Hz (cada 5ms)
- ESP32 actualizado para forward 26 bytes (mantener bypass transparente)
- Rust protocol.rs: parser actualizado para 26 bytes
- Rust serial_comm.rs: manejar nuevo formato
- Rust app.rs: Nueva pestaña "Calibración ToF" con 4 tarjetas:
  1. Configuración del Experimento (L, Umbral, Método)
  2. Control de Secuencia (INICIAR, badges estado, resultados Δt y f1)
  3. Registro de Impacto (Dual Plot: mAngle azul, sAngle naranja)
  4. Función de Transferencia (constantes A/B, botón TEST f1)
- Máquina de estados Rust: Idle→Arming→Silence→Firing→Listening→Done
- Selector de método: Sniper (pulso 50ms 2000µs) y Heartbeat (detección relé)
- Cálculo: Δt = T1-T0, f1 = 1/(2·Δt)

### Must NOT have (guardrails, anti-slop, scope boundaries)
- No modificar Tab 1 existente (DAQ Lazo Abierto)
- No agregar dependencias Rust nuevas (usa eframe/egui/egui_plot existentes)
- No cambiar baud rate (460800) ni intervalo de telemetría existente (15ms)
- No implementar control PID o FSM en microcontroladores
- No agregar WiFi, MQTT, o procesamiento en ESP32
- No cambiar la arquitectura de hilos del serial_comm

## Verification strategy
> Zero human intervention - all verification is agent-executed.
- Test decision: tests-after + cargo build/test
- Evidence: .omo/evidence/task-<N>-tof-calibration.<ext>

## Execution strategy
### Parallel execution waves
> Target 5-8 todos per wave. Fewer than 3 (except the final) means you under-split.

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1. ATmega packet | - | 2, 3 | - |
| 2. ESP32 passthrough | 1 | 3 | - |
| 3. Rust protocol | 2 | 4, 5, 6 | - |
| 4. Rust serial comm | 3 | 5, 6 | - |
| 5. Rust HMI UI (Tab + Cards) | 3, 4 | 6 | - |
| 6. Rust ToF State Machine | 5 | F1-F4 | - |

## Todos
> Implementation + Test = ONE todo. Never separate.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->

### Wave 1: Firmware Layer (ATmega + ESP32)

- [ ] 1. ATmega328P: Expand TelemetryPacket with delta-angle fields
  What to do / Must NOT do:
  - Add `mAngle` (int16) and `sAngle` (int16) fields after sHz in TelemetryPacket struct
  - Calculate delta angle in bldcUpdate(): `delta = current_raw - last_raw`, wrap at ±2048
  - Store delta in new variables `as5600DeltaAngle` and expose via getter
  - Update spiTransact to include new fields (bytes 21-24)
  - Update checksum calculation for 26-byte packet
  - Must NOT change existing field positions or sizes
  Must NOT change BLDC_POLL_MS or BLDC_CALC_MS intervals
  Parallelization: Wave 1 | Blocked by: - | Blocks: 2, 3
  References:
  - src/main.cpp:89-101 (TelemetryPacket struct)
  - src/main.cpp:214-248 (bldcUpdate - AS5600 polling)
  - src/main.cpp:487-500 (spiTransact)
  - src/main.cpp:630-654 (Master loop - packet assembly)
  Acceptance criteria (agent-executable):
  - `arduino-cli compile --fqbn arduino:avr:atmega328p` compiles without errors
  - TelemetryPacket sizeof() == 26
  - New fields mAngle and sAngle are populated with delta values
  QA scenarios:
  - Happy: Compile succeeds, packet size verified
  - Failure: Compilation error due to struct alignment → fix __attribute__((packed))
  Evidence: .omo/evidence/task-1-tof-calibration.txt
  Commit: Y | feat(firmware): expand TelemetryPacket with delta-angle fields for ToF

- [ ] 2. ESP32: Update SPI buffer for 26-byte packet
  What to do / Must NOT do:
  - Change PKT_SIZE from 22 to 26 in ESP32 firmware
  - Update spi_trans.length = PKT_SIZE * 8
  - Keep all other logic unchanged (transparent bypass)
  Must NOT add any processing or filtering
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 3
  References:
  - /home/rorosierra/dev/Solar-Robot-Telemetry---Atacama-Dynamic/ATmega328- Telemetria/src/main.cpp:27 (PKT_SIZE define)
  - /home/rorosierra/dev/Solar-Robot-Telemetry---Atacama-Dynamic/ATmega328- Telemetria/src/main.cpp:70 (spi_trans.length)
  Acceptance criteria (agent-executable):
  - `arduino-cli compile --fqbn esp32:esp32:esp32` compiles without errors
  - PKT_SIZE == 26
  QA scenarios:
  - Happy: Compile succeeds
  - Failure: Buffer overflow → ensure recvbuf/sendbuf are 32-byte aligned (already are)
  Evidence: .omo/evidence/task-2-tof-calibration.txt
  Commit: Y | feat(esp32): update PKT_SIZE to 26 for expanded telemetry

### Wave 2: Rust Protocol Layer

- [ ] 3. Rust protocol.rs: Update parser for 26-byte packet
  What to do / Must NOT do:
  - Change PKT_SIZE from 22 to 26
  - Add `m_angle: i16` and `s_angle: i16` fields to TelemetryPacket struct
  - Update parse() to extract bytes 21-24 as new angle fields
  - Update checksum verification loop (still XOR bytes 0..25)
  Must NOT change existing field parsing order
  Parallelization: Wave 2 | Blocked by: 2 | Blocks: 4, 5, 6
  References:
  - rust_hmi/src/protocol.rs:9-10 (PKT_SIZE, PKT_START)
  - rust_hmi/src/protocol.rs:19-30 (TelemetryPacket struct)
  - rust_hmi/src/protocol.rs:34-70 (parse method)
  Acceptance criteria (agent-executable):
  - `cargo build --release` in rust_hmi/ compiles without errors
  - TelemetryPacket has m_angle and s_angle fields
  - PKT_SIZE == 26
  QA scenarios:
  - Happy: Build succeeds, struct has new fields
  - Failure: Checksum mismatch → verify XOR loop covers bytes 0..25
  Evidence: .omo/evidence/task-3-tof-calibration.txt
  Commit: Y | feat(protocol): expand parser for 26-byte packet with angle fields

- [ ] 4. Rust serial_comm.rs: Handle new packet format
  What to do / Must NOT do:
  - Update PKT_SIZE constant usage (already imported from protocol)
  - No code changes needed if protocol.rs is updated correctly
  - Verify reader_thread still works with 26-byte frames
  Must NOT change baud rate (460800) or thread architecture
  Parallelization: Wave 2 | Blocked by: 3 | Blocks: 5, 6
  References:
  - rust_hmi/src/serial_comm.rs:11 (PKT_SIZE import)
  - rust_hmi/src/serial_comm.rs:63 (PKT_SIZE usage in while loop)
  Acceptance criteria (agent-executable):
  - `cargo build --release` compiles without errors
  - serial_comm.rs uses PKT_SIZE from protocol (no hardcoded 22)
  QA scenarios:
  - Happy: Build succeeds, no hardcoded packet size
  - Failure: Old hardcoded values → grep for "22" in serial_comm.rs
  Evidence: .omo/evidence/task-4-tof-calibration.txt
  Commit: Y | chore(serial): verify PKT_SIZE usage for 26-byte packets

### Wave 3: Rust HMI UI

- [ ] 5. Rust app.rs: Add ToF Calibration tab with 4 cards
  What to do / Must NOT do:
  - Add enum `Tab { DAQ, ToF }` and state variable
  - Add tab bar in CentralPanel before render_plots
  - Create render_tof_tab() method with 4 cards:
    - Card 1: Config (L input 1-2m, Umbral input ticks, Método dropdown Sniper/Heartbeat)
    - Card 2: Control (INICIAR button, status badges, Δt and f1 result labels)
    - Card 3: Dual Plot (mAngle blue, sAngle naranja) using egui_plot
    - Card 4: Transfer Function (A/B inputs, TEST f1 button)
  - Add ToF-specific state fields to SysIdApp struct
  - Must NOT modify existing render_controls or render_plots methods
  Must NOT add new dependencies to Cargo.toml
  Parallelization: Wave 3 | Blocked by: 3, 4 | Blocks: 6
  References:
  - rust_hmi/src/app.rs:40-63 (AcTarget enum pattern)
  - rust_hmi/src/app.rs:68-120 (SysIdApp struct)
  - rust_hmi/src/app.rs:815-1043 (render_plots - egui_plot usage)
  - rust_hmi/src/app.rs:1159-1170 (section helper)
  Acceptance criteria (agent-executable):
  - `cargo build --release` compiles without errors
  - Tab selector visible in UI switching between DAQ and ToF
  - 4 cards rendered in ToF tab with correct widgets
  QA scenarios:
  - Happy: Build succeeds, tabs switch correctly
  - Failure: egui_plot import error → verify egui_plot = "0.29" in Cargo.toml
  Evidence: .omo/evidence/task-5-tof-calibration.txt
  Commit: Y | feat(ui): add ToF Calibration tab with 4 cards

- [ ] 6. Rust app.rs: Implement ToF state machine and experiment logic
  What to do / Must NOT do:
  - Add ToFState enum: Idle, Arming, Silence, Firing, Listening, Done
  - Add ToF state fields to SysIdApp: tof_state, tof_start_time, t0_ms, t1_ms, etc.
  - Implement tof_update() called from poll_serial when in ToF tab
  - State transitions:
    - Idle → Arming: on "INICIAR" click, send ARM command
    - Arming → Silence: after 2s delay
    - Silence → Firing: after 1s delay (Heartbeat: mark T0 on relay ON)
    - Firing → Listening: send pulse (Sniper) or after pulse ends
    - Listening → Done: when sAngle > umbral (T1 detected)
    - Done: calculate Δt = T1-T0, f1 = 1/(2·Δt), unlock UI
  - Update dual plot buffers (live_tof_m_angle, live_tof_s_angle)
  - Implement transfer function: µs = (f1 - B) / A
  - Implement TEST f1 button: send computed µs to ESC
  Must NOT block the UI thread (use non-blocking state machine)
  Must NOT modify existing DAQ state or commands
  Parallelization: Wave 3 | Blocked by: 5 | Blocks: F1-F4
  References:
  - rust_hmi/src/app.rs:195-243 (poll_serial - packet handling pattern)
  - rust_hmi/src/app.rs:319-401 (command sending pattern)
  - rust-upgrade.txt:86-109 (state machine specification)
  Acceptance criteria (agent-executable):
  - `cargo build --release` compiles without errors
  - State machine transitions correctly through all states
  - Δt and f1 calculated and displayed
  - Transfer function applies inverse formula
  QA scenarios:
  - Happy: Build succeeds, state machine logic complete
  - Failure: Borrow checker errors → use RefCell or restructure state
  Evidence: .omo/evidence/task-6-tof-calibration.txt
  Commit: Y | feat(tof): implement state machine and experiment orchestration

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [ ] F1. Plan compliance audit
  Verify all 6 components implemented, packet size 26 bytes, all 4 UI cards present
- [ ] F2. Code quality review
  Check for hardcoded values, proper error handling, no unwrap() in production paths
- [ ] F3. Real manual QA
  Build both firmwares, run `cargo build --release`, verify no compilation errors
- [ ] F4. Scope fidelity
  Confirm no changes to Tab 1, no new dependencies, no WiFi/MQTT added

## Commit strategy
- Wave 1: 2 commits (ATmega firmware, ESP32 firmware)
- Wave 2: 2 commits (protocol.rs, serial_comm.rs)
- Wave 3: 2 commits (UI tabs/cards, state machine)
- Final: Squash or keep as feature branch

## Success criteria
- ATmega328P compiles with 26-byte TelemetryPacket
- ESP32 compiles with PKT_SIZE=26
- Rust HMI compiles with updated parser
- ToF tab visible with 4 functional cards
- State machine orchestrates experiment without blocking UI
- Both Sniper and Heartbeat methods selectable
- Δt and f1 calculated correctly from angle data
