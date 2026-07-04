# Cooperative Steering (Virtual Torque Blending)

The canonical in-repo reference for the VTB cooperative-steering feature on angle-control Teslas:
the physics, the algorithm, how it integrates, the safety boundary, the parameters, and tuning. For
the long-form developer guide, the MADS engagement details, and troubleshooting, see the
[project wiki](https://github.com/FeistyFinn/sunnypilot/wiki).

The whole feature lives in `opendbc/sunnypilot/car/tesla/coop_steering.py` (in the `opendbc_repo`
submodule), plus a small flag-plumbing change in `opendbc/sunnypilot/car/interfaces.py` and the panda
safety model in `opendbc/safety/modes/tesla.h`.

---

## 1. What it is

Virtual Torque Blending (VTB) lets the driver of an angle-control Tesla nudge the steering wheel
against openpilot's commanded angle **without disengaging lateral control**. The driver's torque on
the wheel is read from the EPS column sensor (`EPAS3S_torsionBarTorque`), passed through a deadzone,
scaled into a **steering-angle offset** by a speed-dependent gain derived from the vehicle model,
rate-limited, and then **added** to the angle openpilot was already commanding. The blended angle is
sent on `DAS_steeringControl`. Lateral stays engaged for nudges up to the hands-on / hard-yank limits;
only a sustained firm override, a fast flick, or a hard yank tears off. The feature is Tesla-only,
gated by the **`TeslaCoopSteering`** setting → `CP_SP.flags |= COOP_STEERING` (bit 2).

## 2. Why angle-control needs this

Upstream openpilot supports cooperative steering on most cars by injecting a small EPS *torque*
command on top of the driver's torque — the driver and the controller pull on the same rope, the EPS
sums them, and small corrections feel natural. Angle-control Teslas don't expose that knob. The only
steering interface is the angle request: openpilot tells the EPS *where to put the wheel*, the EPS gets
it there, and the EPS firmware decides what to do if the driver pushes back. The blunt option — "any
driver torque above N Nm → kill lateral" — is jarring at low speed where small nudges are useful
(parking, slow turns, lane positioning around an obstacle), and it cedes the cooperative feel openpilot
drivers expect on torque-control cars.

VTB synthesizes that feel inside the angle domain. A steering-angle command and a steering-torque
command are interchangeable to first order: at steady state a chosen wheel angle implies whatever EPS
torque is needed to hold it. So if the driver pushes with torque `τ`, we translate that into "the
driver wants `Δθ` more angle than openpilot is asking for" and send `θ_openpilot + Δθ`. The EPS
produces whatever torque it needs to land at the blended angle, and the driver feels resistance
proportional to how far their nudge is from openpilot's plan — exactly what cooperative steering on a
torque-control car feels like.

## 3. The physics: torque → curvature → angle, and why 1/v²

VTB's gain falls out of the bicycle model rather than a tuning table:

```
driver torque τ  →  target lateral acceleration a_lat  →  curvature κ  →  steering-wheel angle θ
```

**Step 1 — torque → target lateral acceleration.** Full driver input (the post-deadzone usable range
of 2.0 Nm, from the 0.5 Nm deadzone to the 2.5 Nm soft ceiling) maps to `a_lat = 2.0 m/s²`
(`STEER_OVERRIDE_MAX_LAT_ACCEL`). That target matches Tesla's "comfort" steering feel, and ~2.5 Nm is
about where the Tesla EPS escalates `hands_on_level` to 3 and tears off lateral anyway — so the
algorithm spends the *whole usable torque budget* across the *whole comfortable lateral-accel budget*.

**Step 2 — lateral acceleration → curvature.** From the bicycle model, `a_lat = κ · v²`, so:

```python
curvature = lat_accel / (max(1, vEgo) ** 2)  # 1/m   (get_steer_from_lat_accel)
```

The `max(1, vEgo)` floors speed at 1 m/s so the gain plateaus at low speed instead of going infinite
as `v → 0` (at 0.1 m/s, `2/v²` would demand a 0.5 cm turning radius from a tiny nudge).

**Step 3 — curvature → steering-wheel angle.** The vehicle model inverts curvature to a wheel angle,
accounting for the steering ratio (12:1 on Model 3/Y), the wheelbase (2.89 m), and the understeer slip
factor (a real car needs slightly more wheel angle than the pure kinematic bicycle predicts as speed
rises):

```python
return math.degrees(VM.get_steer_from_curvature(curvature, vEgo, 0))  # deg, roll = 0
```

**Step 4 — where the 1/v² stiffening comes from.** The angle for full driver torque is roughly:

```
θ_full(v) ≈ steer_ratio · (L · a_lat / v²  +  slip · a_lat) · 180/π
```

The kinematic term shrinks as `1/v²`; the slip term is speed-independent. At low speed the kinematic
term dominates → the gain is huge. At high speed it collapses → the small, roughly-flat slip term is
what's left. The torque-to-angle gain is then `θ_full(v) / 2.0 Nm`:

| Speed          | Gain (VM-computed) | Felt behavior                          |
|----------------|--------------------|----------------------------------------|
| ~1 m/s (floor) | ~180 °/Nm (angle-capped) | Big wheel motion for a tiny nudge |
| ~5 m/s         | ~81 °/Nm           | Easy parking-lot corrections           |
| ~15 m/s        | ~10 °/Nm           | Lane-positioning nudges feel solid     |
| ~30 m/s        | ~3.4 °/Nm          | Highway: small nudge, small offset     |

These are the actual vehicle-model gain `clip(get_steer_from_lat_accel(2.0, v, VM), 360°) / 2.0 Nm`, **not**
a literal `1/v²` schedule: the understeer slip term makes the gain deviate ~26 % from `1/v²` by 30 m/s
(measured ratios g₅/g₁₅ = 8.08 vs 9.0 predicted, g₁₅/g₃₀ = 2.97 vs 4.0). Below ~3.3 m/s the 360° angle
cap pins the gain flat at 180 °/Nm. You get speed-aware stiffness for free from the vehicle model, with no
gain schedule.

**Torsion-bar torque vs EPS motor torque.** VTB reads `EPAS3S_torsionBarTorque` — the twist in the
column's torsion bar, i.e. **driver intent** at the column — not the EPS motor torque. If it read motor
torque, every angle command openpilot issued would register as a "driver nudge" because the motor was
working to hit it.

## 4. The algorithm, frame by frame

The per-frame work is in `CoopSteeringCarController.update()` / `update_override_angle()`, called every
other carcontroller frame (50 Hz, every 20 ms):

```
τ → deadzone → θ_target = τ · gain(v) → rate-limit toward θ_target
              → subtract planner same-direction delta
              → integrate into angle_override
              → blend: apply_angle + angle_override
              → final saturator (panda-matched, apply_steer_angle_limits_vm)
              → unwind override by sat_error (anti-windup)
              → emit blended angle on DAS_steeringControl
```

Key points:

- **Feature gate.** If `COOP_STEERING` is clear or `lat_active` is false, the override state is reset
  (`reset_override_state`) and the commanded angle passes through unchanged.
- **Engagement ramp** (`resume_steer_desired_rate_limit`). On re-engage the command is
  acceleration-limited to `STEER_RESUME_RATE_LIMIT_RAMP_RATE = 300 °/s²` so the wheel never snaps.
- **Continuous deadzone** (`apply_deadzone`): zero output for `|τ| ≤ 0.5 Nm`, then `τ − sign(τ)·0.5`
  above it — no step at the boundary.
- **Holding-torque estimate — and its inversion at speed.** `holding_torque = angle_override /
  torque_to_angle` (the §3 gain; forced to 0 below 0.1 m/s) — "what steady nudge would the current
  override correspond to?" — feeds the centering rate. Because the gain *shrinks* with speed, holding a
  **fixed** angle offset costs **more** sustained torque the faster you go: a 5° offset that is
  effectively free to hold in a parking lot (≈0.03 Nm at 1 m/s, ≈0.06 at 5) costs ≈0.50 Nm at 15 m/s
  (right at the 0.5 Nm deadzone floor) and **≈1.49 Nm at 30 m/s** — well into the usable band. This is the
  `1/v²` stiffening felt as holding effort; it is expected and safe, but it means the estimate is
  noise-sensitive at high speed (small override noise → large holding-torque swings feeding the centering
  rate).
- **Dual-gain rate limit.** Away-from-center and centering deltas are bounded per frame; both currently
  equal 125 °/s/Nm because both bump the same `MAX_ANGLE_RATE = 5°/20 ms` cap
  (`125 == MAX_ANGLE_RATE / DT_LAT_CTRL / STEER_OVERRIDE_TORQUE_RANGE`). The asymmetric hook is in the
  code for future tuning; it is numerically neutralized today.
- **Double-count avoidance (planner-direction overlap).** When the planner's own per-frame angle change
  and the driver-override delta point the **same way** (`angle_override_delta · apply_angle_delta > 0`),
  the planner is already moving the wheel where the driver is nudging, so the override delta is reduced by
  the overlapping planner delta — bounded to `abs(angle_override_delta)`, so it can neither flip sign nor
  over-subtract. The override then integrates only the driver's *marginal* contribution beyond what the
  planner already delivers. When the driver pulls **against** the planner, or either delta is zero
  (product ≤ 0), **no subtraction** happens and the nudge keeps full authority. Across a planner
  **reversal** the subtraction toggles on/off frame-to-frame, but because it is bounded by
  `abs(override_delta)` the override stays **continuous** (no step) through the reversal. This
  opposite-pull behavior is a deliberate design choice, not incidental.
- **Integration + final saturation.** The override is an integrator; the blended angle is run through
  the same vehicle-model angle limiter the panda enforces (`apply_steer_angle_limits_vm`).
- **Anti-windup unwind** (`unwind_override_angle_progressive`). If the final saturator clipped the
  request in the override's direction, the override is reduced by exactly that error so it doesn't
  integrate against the safety wall.

## 5. Integration map

VTB is a thin, post-hoc transform on the angle the car-controller was already about to TX. It touches
no planner, no `LatControlAngle`, no cereal struct, no upstream daemon.

```
        Tesla CAN (party)                          carcontroller (50 Hz, Tesla)
   ┌────────────────────────┐                 ┌──────────────────────────────────┐
   │ EPAS3S_sysStatus 0x370 │  steeringTorque │ apply_angle = apply_steer_angle_  │
   │   torsionBarTorque     │ ───────────────▶│   limits_vm(planner angle)        │  ← 1st saturation
   │   handsOnLevel         │  steeringAngle  │ ────────────────────────────────  │
   │   eacStatus            │  vEgo           │ coop_steer.update(apply_angle, …): │  ← reads CS torque,
   │ DI_speed 0x257         │                 │   τ → angle_override → blend       │     vEgo, CP_SP.flags
   └────────────────────────┘                 │   apply_steer_angle_limits_vm     │  ← 2nd saturation
                                              │ tesla_can.create_steering_control │
                                              └────────────────┬─────────────────┘
                                                               ▼
                                              DAS_steeringControl 0x488 (50 Hz)
                                                               ▼
                                              ┌──────────────────────────────────┐
                                              │ panda firmware (tesla.h)          │
                                              │   3 disengage paths               │
                                              │   angle / rate / jerk bounds      │
                                              └────────────────┬─────────────────┘
                                                               ▼
                                                    Tesla EPAS (angle control)
```

The **double saturation** is intentional: the first call bounds the *unblended* planner command; the
second bounds the *blended* command (planner + driver nudge). The flag is set once at car-init in
`_initialize_coop_steering()`:

```python
if CP.brand == 'tesla' and int(params_dict.get("TeslaCoopSteering", 0)) == 1:
  CP_SP.flags |= TeslaFlagsSP.COOP_STEERING.value          # bit 2
  CP_SP.teslaCoopSteeringInertiaJ = float(params_dict.get("TeslaCoopSteeringInertiaJ", 0.0))
```

so the algorithm reads `CP_SP.flags` every frame to gate itself. The UI toggle is in the Tesla brand
settings panel as **"Cooperative Steering"** (offroad-gated).

## 6. The safety boundary

The panda safety model in `opendbc/safety/modes/tesla.h` bounds VTB regardless of what
`coop_steering.py` does. Lateral disengages on the **OR of three independent triggers** — they do not
all fire at the same torque:

| Trigger | Threshold | Fires first when… |
| --- | --- | --- |
| **Hands-on level** | `hands_on_level ≥ 3` | A sustained, gentle override (~0.25 s above ~0.5 Nm; typically ~2.5 Nm). **The first path a steady firm nudge trips — expected, not a fault.** |
| **EAC angle-rate fault** | `eac_status == 0 && eac_error_code == 9` | A **fast flick** — the EPS rejects a high angle-rate request, independent of torque magnitude. |
| **Torsion-bar torque** | `|torsion_bar_torque| > 5.0 Nm` | A slow, deliberate **hard yank** (strictly greater-than: ±5.00 Nm does *not* disengage, ±5.01 Nm does). |

Disengage is **latched** — removing the override does not re-engage; controls must be re-enabled. On
Tesla, a single **brake** press also disengages lateral, on a path independent of the steering/torque
chain.

**TX angle / rate bounds.** The angle ceiling is 360° (the EPAS faults above it). The commanded
angle's *rate* is bounded by ISO-11270-style lateral-accel and jerk limits via the vehicle model, so
the allowed per-frame angle delta tightens as speed climbs. VTB's own rate limiter is set to bump the
same `MAX_ANGLE_RATE = 5°/frame`, so the algorithm cannot construct a command that would TX-violate.

**Gas override (cooperative longitudinal).** `longitudinal_allowed_on_gas = true` is set at init, so
pressing the accelerator does not stop openpilot from publishing longitudinal commands; the long PID
integrator freezes during the press and resumes on lift-off. Longitudinal accel bounds (+2.0 / −3.48 /
0.0 m/s²) and reverse-prevention are still enforced. Steering blends, gas blends; **brake still
disengages.**

**What changed vs upstream, and why it's safe.** The VTB delta's `opendbc/safety/` surface has four
independent parts, each defensible on its own:

1. **Torsion-bar disengage (added).** The `|torsion_bar_torque| > 5.0 Nm` trigger above is a *new*
   OR-chained disengage path (const `TESLA_STEERING_DISENGAGE_TORQUE = 500` cNm), with a MISRA C:2012 fix
   to its decode. It only *adds* a disengage condition, so the set of states in which lateral stays
   engaged is a **strict subset** of upstream's — strictly safer. It is the hard-yank backstop: a
   sustained firm nudge trips hands-on (~2.5 Nm) first; the 5 Nm path catches a slow, deliberate yank
   that outruns the hands-on debounce.
2. **Valid-TX-type narrowing.** Base cooperative steering transmits **only** angle control
   (`DAS_steeringControl`). Upstream's valid-TX allow-list also permits LKAS-style steering TX; since coop
   never emits LKAS torque, keeping LKAS in the allow-list leaves an unused, unbounded TX path. Narrowing
   the allow-list to NONE + ANGLE_CONTROL removes that latent path — strictly fewer allowed TX types, and
   *required by* coop's pure-angle design.
3. **`autopark → summon`.** A naming/semantics fix (the `tesla_autopark` → `tesla_summon` rename;
   stock-steering detection broadened from `== LANE_KEEP_ASSIST` to `!= NONE`) so Autopilot-enabled /
   autopark states don't spuriously disengage. It loosens no bound. *(Contributed by Amy Jeanes; lands as
   its own PR.)*
4. **Gas-override RX swap.** Removed the dedicated `DI_systemStatus` (0x118) RX-check that read a
   multi-bit pedal magnitude, replacing it with a single `DI_accelPedalPressed` bit on the
   already-RX-checked `DI_speed` (0x257): one fewer independent RX-checked message and one fewer signal to
   validate, with a binary pressed/not-pressed flag that's less ambiguous than a magnitude+threshold (the
   gas-override gate is a one-time init flag, so a boolean suffices). *(Ships with the separate
   gas-override feature.)*

The Tesla safety suite (`test_tesla.py`) is the gate for all four and stays green (353 passed / 234
subtests as of this writing; boundary cases in `test_steering_wheel_torque_disengage` and
`test_autopark_summon_while_enabled` / `test_autopark_summon_behavior`).

## 7. Parameters

| Param | Type / default | Gating | Effect |
| --- | --- | --- | --- |
| `TeslaCoopSteering` | bool, `0` | offroad-only toggle | Master enable; sets `CP_SP.flags COOP_STEERING` (bit 2) at car-init. |
| `TeslaInfotainmentMadsToggleFingers` | int, `5` (3 / 4 / 5) | offroad-only toggle | MADS infotainment-tap finger count; threaded to both openpilot (`flags`) and panda (`safetyParam`). See the wiki MADS page. |
| `TeslaCoopSteeringInertiaJ` | float, `0.0` → module default `0.08` (clamp `0.15`) | param only | Scales the **logged** inertia feed-forward only — **shadow-only, never steers.** See §8. |

## 8. Inertia compensation (shadow-only) and tuning

The algorithm computes a feed-forward term `tau_inertia = J · alpha_wheel` (filtered steering-column
angular acceleration) every frame — the component of measured torque that goes into accelerating the
wheel's own inertia rather than expressing intent — and logs it alongside the reconstructed
`tau_intent = τ_measured − tau_inertia`. **This is shadow-only: it is always computed and logged, but
never applied to steering** — the live override always runs off raw measured torque. `J` defaults to
0.08 kg·m² (literature band 0.05–0.15), is hard-clamped at 0.15, and is field-tunable via
`TeslaCoopSteeringInertiaJ`, which scales the *logged* term only. The live-apply path and its flag bit
were removed pending a validated J; the offline fit consumes the logged telemetry
(`tools/sunnypilot/vtb/fit_steer_inertia.py`).

**Tunable steering constants** (top of `coop_steering.py`). VTB cannot escape
`apply_steer_angle_limits_vm`, so a more aggressive setting only buys what the safety envelope allows:

| Constant | Current | Unit | Increase → effect | Watch |
| --- | --- | --- | --- | --- |
| `STEER_OVERRIDE_MIN_TORQUE` | 0.5 | Nm | Bigger deadzone, fewer false triggers | Above typical bias+noise, light pressure won't engage |
| `STEER_OVERRIDE_MAX_TORQUE` | 2.5 | Nm | Wider usable range before saturating | Past ~2.5 Nm the EPS hands-on path fires before you use the range |
| `STEER_OVERRIDE_MAX_LAT_ACCEL` | 2.0 | m/s² | More angle per Nm at every speed | At speed the angle runs into panda's lateral-accel/jerk bounds |
| `STEER_OVERRIDE_TARGET_ANGLE_MAX` | 360 | deg | (At the EPS fault threshold — don't raise) | EPAS faults above 360°; lowering is fine |
| `STEER_OVERRIDE_DELTA_GAIN_LIMIT` | 125 | °/s/Nm | Faster ramp building the override | Capped by `MAX_ANGLE_RATE / DT / RANGE = 125`; no effect raising it alone |
| `STEER_OVERRIDE_DELTA_GAIN_LIMIT_CENTERING` | 125 | °/s/Nm | Faster return-to-center | *Derived* from the same cap; change the formula to move it |
| `STEER_RESUME_RATE_LIMIT_RAMP_RATE` | 300 | °/s² | Snappier engagement after disengage | Too high feels like a jerk on re-engage |
| `MAX_ANGLE_RATE` (coop) | 5 | °/20 ms | Larger per-frame delta, sharper feel | Must stay ≤ what panda enforces; moves the centering gain and per-Nm cap together |

Change one knob at a time and watch for: the override actually moving on a deliberate nudge; a clean
return to the planner's path on release with no residual offset; no disengage from routine tuning
torque; and no "fight" oscillation. Re-run the tests below before driving.

## 9. Testing

**`test_coop_steering.py`** pins the algorithm: pure helpers (`apply_bounds`, `apply_deadzone`,
direction), feature gating (passthrough when the flag is clear / `lat_active` is false), deadzone
(0.3 Nm → zero override), direction (positive torque → positive override/angle), monotonic ramp (a
one-frame step is a fraction of the settled override), hard saturation at 360° under absurd torque, and
clean release (reset to zero on disengage).

**`test_tesla.py`** pins the panda safety boundary: `test_steering_wheel_disengage` (the hands-on and
EAC paths over every relevant combination, with latched no-auto-recovery), `test_steering_wheel_torque_disengage`
(the ±5.00 / ±5.01 Nm boundary), `test_no_disengage_on_gas`, and `test_prevent_reverse` (accel bounds +
reverse-prevention). Run both from inside the submodule:

```bash
cd opendbc_repo && pytest opendbc/safety/tests/test_tesla.py \
                          opendbc/sunnypilot/car/tesla/tests/test_coop_steering.py
```

**Not covered** (correct-by-review, not by test): an end-to-end replay through controlsd → carcontroller
with VTB engaged; the planner-interaction (double-count) branch under a moving planner; and high-speed
behavior (all unit tests use `v_ego = 5 m/s`). These are the places to add coverage when touching the
algorithm.

## 10. Source-file index

- Algorithm: `opendbc/sunnypilot/car/tesla/coop_steering.py`
- Algorithm tests: `opendbc/sunnypilot/car/tesla/tests/test_coop_steering.py`
- Flag / param plumbing: `opendbc/sunnypilot/car/interfaces.py` (`_initialize_coop_steering`,
  `_initialize_tesla_infotainment_gesture`), flags in `opendbc/sunnypilot/car/tesla/values.py`
- Car interface: `opendbc/car/tesla/{carcontroller,carstate,teslacan,values}.py`
- Panda safety: `opendbc/safety/modes/tesla.h`; tests `opendbc/safety/tests/test_tesla.py`
- Offline tooling: `tools/sunnypilot/vtb/` (`fit_steer_inertia.py`, `live_watch.py`,
  `transcribe_events.py`, `analyze_shadow.py`)
