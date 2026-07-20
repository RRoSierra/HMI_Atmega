# tof-hardware-detection - Draft

## Status: awaiting-approval

## Pending action: write .omo/plans/tof-hardware-detection.md (already written, awaiting user approval)

## Evaluation of Recommendations

### Recommendation 1: Remove `.abs()` from T1 delta calculation
**Verdict: KEEP `.abs()` (direction-agnostic detection)**

The recommendation claims `.abs()` allows negative "falling edge" spikes to trigger T1. However:
- With a clean baseline (FIXED in prior turn via Arming collection), `abs(0 - 0) = 0 < threshold` → falling edge does NOT trigger
- The root cause of the falling-edge trigger was baseline contamination (~500), which we already fixed
- Removing `.abs()` would make the system blind to negative-direction wave impacts (wave causing rotation in negative direction)
- For a rope-cleaning robot, wave propagation can cause rotation in either direction depending on pulse polarity
- **Recommendation**: Keep `.abs()` for robustness. The baseline fix already solves the falling-edge problem.

### Recommendation 2: Hardware-based T0 (encoder-driven)
**Verdict: IMPLEMENT — this is the real improvement**

Current T0 (line 682): `self.tof_t0_ms = self.last_packet.as_ref().map(|p| p.timestamp_ms)`
- Set when the serial command is sent, NOT when the encoder actually moves
- Does not account for ESC inertia, serial latency, or motor response time
- T0 appears before the encoder shows movement on the graph → inaccurate ToF calculation

Proposed T0: Detect when shooter encoder delta exceeds shooter baseline + threshold
- Requires a `tof_baseline_shooter` (analogous to `tof_baseline` for receiver)
- Collect shooter baseline during Arming state (before ESC spins)
- During Firing, detect shooter delta spike → this is the real T0

### Recommendation 3: Shooter baseline during Arming
**Verdict: IMPLEMENT — required for Recommendation 2**

Need to add:
- `tof_baseline_shooter: f64` field to SysIdApp struct
- `tof_baseline_shooter_samples: VecDeque<f64>` for collection
- Collect shooter delta during Arming (same as receiver, but different encoder)
- Compute shooter baseline at Silence→Firing transition

### Recommendation 4: Detect both T0 and T1 from encoder telemetry
**Verdict: IMPLEMENT — this is the unified approach**

- T0: Shooter delta exceeds shooter baseline + threshold (encoder-driven)
- T1: Receiver delta exceeds receiver baseline + threshold (already works)
- Both baselines collected during Arming, computed at Silence→Firing transition

## Topology

| Component | Outcome | Status | Evidence |
|-----------|---------|--------|----------|
| Shooter baseline collection | Clean baseline before ESC spins | Needs implementation | `app.rs:404-411` (receiver baseline pattern) |
| Shooter baseline computation | Average last 5 samples at Silence→Firing | Needs implementation | `app.rs:668-676` (receiver baseline pattern) |
| T0 hardware detection | Detect shooter delta spike during Firing | Needs implementation | `app.rs:682` (current software-based T0) |
| T1 detection (keep `.abs()`) | Already works, keep as-is | Verified | `app.rs:414-448` |
| Struct fields | Add `tof_baseline_shooter` + samples | Needs implementation | `app.rs:226-227` (receiver baseline fields) |
| Constructor init | Initialize new fields | Needs implementation | `app.rs:333-334` (receiver baseline init) |
| tof_arm() reset | Clear shooter baseline samples | Needs implementation | `app.rs:721-742` |

## Open Decisions

1. **T0 threshold**: Use same `tof_threshold` (10.0) for both T0 and T1, or separate `tof_threshold_shooter`? → Default: same threshold (simpler, works for most cases)
2. **`.abs()` on T1**: Keep `.abs()` for direction-agnostic detection, or remove it for rising-edge-only? → Default: keep `.abs()` (safer, baseline fix already solves falling-edge)

## Approach
1. Add shooter baseline fields to struct + constructor
2. Collect shooter baseline during Arming
3. Compute shooter baseline at Silence→Firing transition
4. Modify detection block to detect T0 from shooter encoder
5. Keep T1 detection as-is (with `.abs()`)
6. Update graph rendering if needed (T0 line position changes from software to hardware)
