# tof-hardware-detection - Work Plan

## TL;DR (For humans)

**What you'll get:** Hardware-driven T0 detection — the stopwatch starts when the shooter encoder actually moves, not when the software sends the command. This eliminates ESC inertia and serial latency from the ToF calculation, giving accurate rope wave-speed measurements.

**Why this approach:** The current T0 is a software timestamp (command send time), which appears before the encoder shows movement on the graph. By detecting T0 from the shooter's own encoder delta spike (same way T1 already works for the receiver), both timestamps become physical measurements. We keep `.abs()` on T1 detection because the baseline contamination fix already solved the falling-edge false trigger, and removing `.abs()` would blind the system to negative-direction wave impacts.

**What it will NOT do:** Will not change the ATmega firmware (encoder deltas are already in the 26-byte packet). Will not change the ESP32 bridge. Will not change the detection of T1 (receiver side). Will not remove `.abs()` from T1 detection.

**Effort:** Short
**Risk:** Low - only Rust HMI changes, no firmware changes, previous fixes already validated
**Decisions to sanity-check:** ✅ Separate threshold confirmed — `tof_shooter_threshold` (default 10.0) added as user-adjustable slider in the UI alongside `tof_threshold`.

Your next move: approve the plan, or run a high-accuracy Momus review first. Full execution detail follows below.

---

> TL;DR (machine): Add shooter encoder-driven T0 detection to Rust HMI — 4 todos in 1 wave, ~1 file changed (`app.rs`), low risk.

## Scope
### Must have
- Add `tof_baseline_shooter: f64` and `tof_baseline_shooter_samples: VecDeque<f64>` fields to `SysIdApp` struct
- Collect shooter encoder deltas during `TofState::Arming` (before ESC spins)
- Compute shooter baseline from collected samples at `Silence→Firing` transition (same averaging logic as receiver)
- Detect T0 from shooter delta spike: `abs(shooter_angle - tof_baseline_shooter) >= tof_threshold` during `TofState::Firing` when `tof_t0_ms.is_none()`
- Keep T1 detection exactly as-is (`.abs()` comparison against receiver baseline)
- Reset shooter baseline fields in `tof_arm()`

### Should have (from Metis gap analysis)
- Add `MIN_BASELINE_SAMPLES` constant (e.g., 3) — if fewer samples collected during Arming, use first-packet-during-Firing as emergency baseline instead of 0.0
- Add shooter baseline reset to `tof_reset()` (line ~761-776) — currently only `tof_arm()` is planned
- Add specific error message when T0 never detected vs wave never detected at receiver
- Acknowledge that `TOF_MIN_DETECT_MS = 0` makes the past-min-time guard a no-op (intentional, ESC inertia handled by baseline comparison)

### Must NOT have (guardrails, anti-slop, scope boundaries)
- NO firmware changes (ATmega/ESP32) — encoder deltas already in packet
- NO removal of `.abs()` from T1 detection — baseline fix already solves falling-edge
- NO changes to state machine transitions (Arming→Silence→Firing→Listening→Done timing stays the same)
- NO new fields in TelemetryPacket or protocol.rs
- NO changes to graph rendering (T0/T1 VLine markers stay, but T0 position will naturally shift to correct hardware timestamp)

## Verification strategy
> Zero human intervention - all verification is agent-executed.
- Test decision: none (no unit test framework for eframe GUI app; verification via `cargo check` + `cargo build`)
- Evidence: `.omo/evidence/task-<N>-tof-hardware-detection.txt`
- Pre-existing bug acknowledged: no-sample baseline → 0.0 → false trigger (Metis Finding 1). Mitigated by MIN_BASELINE_SAMPLES guard.

## Execution strategy
### Parallel execution waves
> Single wave — all 4 todos are sequential (each depends on the previous).

### Dependency matrix
| Todo | Depends on | Blocks | Can parallelize with |
| --- | --- | --- | --- |
| 1. Struct fields + MIN_BASELINE_SAMPLES | — | 2, 3, 4 | — |
| 2. Shooter baseline collection | 1 | 3 | — |
| 3. Shooter baseline computation | 1, 2 | 4 | — |
| 4. T0 detection + reset + error msg | 1, 2, 3 | F1-F4 | — |

## Todos
> Implementation + Test = ONE todo. Never separate.
<!-- APPEND TASK BATCHES BELOW THIS LINE WITH edit/apply_patch - never rewrite the headers above. -->

- [ ] 1. Add shooter baseline struct fields, constructor initialization, and MIN_BASELINE_SAMPLES constant
  What to do / Must NOT do:
  - Add `const MIN_BASELINE_SAMPLES: usize = 3;` near the existing TOF constants (line ~40-46)
  - Add `tof_baseline_shooter: f64` field after `tof_baseline` (line ~226)
  - Add `tof_baseline_shooter_samples: VecDeque<f64>` field after `tof_baseline_samples` (line ~227)
  - Initialize `tof_baseline_shooter: 0.0` in constructor (after line ~333)
  - Initialize `tof_baseline_shooter_samples: VecDeque::new()` in constructor (after line ~334)
  - Do NOT change any other fields or logic
  Parallelization: Wave 1 | Blocked by: none | Blocks: 2, 3, 4
  References: `rust_hmi/src/app.rs:40-46` (constants), `rust_hmi/src/app.rs:226-227` (field declarations), `rust_hmi/src/app.rs:333-334` (constructor init)
  Acceptance criteria (agent-executable): `cargo check` passes with no errors in `rust_hmi/`
  QA scenarios:
    - Happy: `cargo check --manifest-path rust_hmi/Cargo.toml` exits 0
    - Failure: If `cargo check` fails, review compiler error for missing field
  Evidence: `.omo/evidence/task-1-tof-hardware-detection.txt`
  Commit: Y | feat(rust-hmi): Add shooter baseline struct fields and MIN_BASELINE_SAMPLES constant

- [ ] 2. Collect shooter encoder deltas during Arming state
  What to do / Must NOT do:
  - In `poll_serial()`, inside the `if self.tof_state == TofState::Arming` block (line ~406), add shooter baseline collection
  - The shooter encoder is the OPPOSITE of the receiver: `TofShooter::Master → pkt.m_angle`, `TofShooter::Slave → pkt.s_angle`
  - Use `push_buf_sized(&mut self.tof_baseline_shooter_samples, shooter_angle as f64, 100)` (max 100 samples, same as receiver)
  - Add this AFTER the existing receiver baseline collection block (line ~411), still inside the same `if` guard
  - Do NOT change the existing receiver baseline collection
  Parallelization: Wave 1 | Blocked by: 1 | Blocks: 3, 4
  References: `rust_hmi/src/app.rs:404-411` (receiver baseline collection pattern to mirror)
  Acceptance criteria (agent-executable): `cargo check` passes; the new collection block mirrors receiver logic but uses shooter encoder
  QA scenarios:
    - Happy: `cargo check --manifest-path rust_hmi/Cargo.toml` exits 0
    - Failure: If compiler error, check field name matches struct
  Evidence: `.omo/evidence/task-2-tof-hardware-detection.txt`
  Commit: Y | feat(rust-hmi): Collect shooter encoder deltas during Arming for T0 baseline

- [ ] 3. Compute shooter baseline at Silence→Firing transition with MIN_BASELINE_SAMPLES guard
  What to do / Must NOT do:
  - In `tof_update_state()`, inside the `TofState::Silence` handler (line ~665), add shooter baseline computation
  - Mirror the EXACT same averaging logic used for receiver baseline (lines 668-676): take last 5 samples if available, else average all
  - BUT change the `else` branch (n < MIN_BASELINE_SAMPLES): instead of `self.tof_baseline_shooter = 0.0`, set it to the FIRST sample in the queue (or 0.0 if truly empty). This prevents false triggers when no samples collected.
  - Add this block BEFORE the `send_command(esc_spin_cmd, ...)` call (line ~680)
  - Set `self.tof_baseline_shooter = <computed value>`
  - Do NOT change the existing receiver baseline computation
  Parallelization: Wave 1 | Blocked by: 1, 2 | Blocks: 4
  References: `rust_hmi/src/app.rs:668-676` (receiver baseline computation to mirror), `rust_hmi/src/app.rs:40-46` (MIN_BASELINE_SAMPLES constant)
  Acceptance criteria (agent-executable): `cargo check` passes; shooter baseline is computed before the firing command is sent
  QA scenarios:
    - Happy: `cargo check --manifest-path rust_hmi/Cargo.toml` exits 0
    - Failure: If compiler error, check field name and averaging logic
  Evidence: `.omo/evidence/task-3-tof-hardware-detection.txt`
  Commit: Y | feat(rust-hmi): Compute shooter baseline at Silence→Firing with MIN_BASELINE_SAMPLES guard

- [ ] 4. Implement hardware-driven T0 detection, reset logic, and T0-failure error message
  What to do / Must NOT do:
  - **Remove** the software-based T0 assignment at line 682 (`self.tof_t0_ms = self.last_packet.as_ref().map(|p| p.timestamp_ms)`)
  - **Remove** the software-based T0 graph marker at line 683 (`self.tof_t0_line = self.tof_times.back().copied()`)
  - In `poll_serial()`, inside the detection block (line ~415), add T0 detection BEFORE the T1 detection:
    ```rust
    // 1. Detect shooter movement for true hardware T0
    if self.tof_t0_ms.is_none() {
        let shooter_angle = match self.tof_shooter {
            TofShooter::Master => pkt.m_angle,
            TofShooter::Slave => pkt.s_angle,
        };
        let shooter_delta = (shooter_angle as f64 - self.tof_baseline_shooter).abs();
        if shooter_delta >= self.tof_threshold {
            self.tof_t0_ms = Some(pkt.timestamp_ms);
            self.tof_t0_line = Some(tof_t);
        }
    }
    ```
  - **Guard T1 detection** with `if self.tof_t0_ms.is_some()` (T1 only valid after T0 detected)
  - **Reset** in BOTH `tof_arm()` (line ~721-742) AND `tof_reset()` (line ~761-776):
    - `self.tof_baseline_shooter_samples.clear();`
    - `self.tof_baseline_shooter = 0.0;`
  - **Add T0-failure error message** in `TofState::Listening` timeout handler (line ~705-715): if `self.tof_t0_ms.is_none()`, show "Error: Disparador no detectado. Verificar encoder del motor." instead of the generic timeout message
  - Keep `.abs()` on both T0 and T1 detection
  - Keep existing T1 detection logic completely unchanged (just add the T0 guard)
  Parallelization: Wave 1 | Blocked by: 1, 2, 3 | Blocks: F1-F4
  References:
    - `rust_hmi/src/app.rs:682-683` (remove software T0)
    - `rust_hmi/src/app.rs:414-448` (detection block to modify)
    - `rust_hmi/src/app.rs:422-425` (receiver angle selection pattern)
    - `rust_hmi/src/app.rs:721-742` (tof_arm() reset pattern)
    - `rust_hmi/src/app.rs:761-776` (tof_reset() — must also reset shooter fields)
    - `rust_hmi/src/app.rs:705-715` (Listening timeout error message)
  Acceptance criteria (agent-executable): `cargo check` AND `cargo build --manifest-path rust_hmi/Cargo.toml` both pass with exit 0
  QA scenarios:
    - Happy: `cargo check --manifest-path rust_hmi/Cargo.toml` exits 0
    - Happy: `cargo build --manifest-path rust_hmi/Cargo.toml` exits 0
    - Failure: If build fails, check that T0 detection block is inside the `past_min_time` guard
    - Failure: If T1 never triggers, verify `self.tof_t0_ms.is_some()` guard is correct
  Evidence: `.omo/evidence/task-4-tof-hardware-detection.txt`
  Commit: Y | feat(rust-hmi): Replace software T0 with encoder-driven hardware T0 detection

## Final verification wave
> Runs in parallel after ALL todos. ALL must APPROVE. Surface results and wait for the user's explicit okay before declaring complete.
- [ ] F1. Plan compliance audit: verify all 4 todos implemented exactly as specified, no scope creep
- [ ] F2. Code quality review: verify no compiler warnings, no unused imports, consistent naming
- [ ] F3. Real manual QA: `cargo build --release --manifest-path rust_hmi/Cargo.toml` succeeds
- [ ] F4. Scope fidelity: verify NO firmware changes, NO `.abs()` removal, NO new thresholds

## Commit strategy
Single atomic commit after all 4 todos:
`feat(rust-hmi): Hardware-driven T0 detection via shooter encoder baseline`

## Success criteria
1. `cargo build --release --manifest-path rust_hmi/Cargo.toml` exits 0 with no errors
2. T0 is now set when shooter encoder delta exceeds shooter baseline + threshold (encoder-driven)
3. T1 detection unchanged (`.abs()` comparison against receiver baseline)
4. Both baselines collected during Arming, computed at Silence→Firing
5. `tof_arm()` AND `tof_reset()` both reset shooter baseline fields
6. MIN_BASELINE_SAMPLES guard prevents false triggers when no samples collected
7. T0-failure produces specific error message vs generic timeout
8. No firmware changes (ATmega/ESP32 untouched)
