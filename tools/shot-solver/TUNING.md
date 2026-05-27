# Tuning the shot-solver

This doc covers the three sliders that don't come from physics first principles:

- `η` (slip efficiency)
- `wear` (ball condition)
- `angle bias` (hood ↔ ball-launch offset)

If your physical constants (mass, diameter, Cd, ρ, g, hub geometry, launch
height) are correct, these three knobs are the only things you should be
touching to make the solver agree with the robot.

---

## What each knob actually is

### η × wear (the transfer ratio)

The flywheel surface velocity `v_wheel = ω·r` is the upper bound on ball exit
velocity. The real shooter loses energy to compression, spin pickup, short
contact dwell. The model collapses all of that into one product:

```
v_ball = (η × wear) × v_wheel
```

`η` and `wear` are mathematically the same knob. The split exists so that:

- `η` captures the **shooter** (wheel hardness, compression preload, dwell).
  Fixed until you change hardware.
- `wear` captures **today's ball** (fresh hard ≈ 1.0, soft worn ≈ 0.85). Re-tune
  per event.

If you only ever have one ball, leave `wear = 1.0` and put everything in `η`.

### angle bias (degrees)

A uniform offset added to the physics-optimal launch angle to produce the hood
command in the LUT:

```
hood_command_deg = physics_optimal_deg + angle_bias
```

Use it when slow-mo shows the ball leaves the shooter at a flatter (or steeper)
angle than the hood encoder reads. **It is a constant offset.** If the residual
between solver and empirical hood° has a *shape* across distance, no value of
this knob will fix it — see "What this tool can't do" below.

---

## How to measure η empirically

You need one shot with a chronograph or high-speed video, and the commanded
wheel speed.

1. Pick a known-good ball (the one you want to call your reference).
2. Command a wheel surface velocity `v_wheel` you can read off the dashboard.
   Wheel surface m/s = `motor_rev_per_sec × (2π × wheel_radius)`.
3. Measure the ball exit velocity `v_ball` immediately after the shooter.
4. `η = v_ball / v_wheel`. Set `wear = 1.0`.

Sanity check: a typical FRC compliant-wheel shooter lands in the **0.5–0.8**
range. If you compute something outside that, you measured the wrong thing —
double-check that `v_wheel` is surface velocity (not motor RPS) and that
`v_ball` is right at exit (not after several meters of drag).

Re-measure with a worn ball to get `wear`:

```
wear = (worn_ball v_ball / v_wheel) / η
```

You can't separate η from wear in a single shot — only their product is
observable. The split only earns its keep when you need to retune for ball
changes without re-deriving everything else.

---

## How to tune against an existing empirical LUT

If you have a known-good LUT (e.g. `loadLUTSoft` in `ShooterStructure.java`)
that scores reliably, you can use it to back out η × wear without a chrono.

### Step 1 — find a distance where the angles already agree

Generate `loadLUTSolved` with your current sliders, then line up against the
empirical LUT distance-by-distance:

```
empirical_θ vs solver_θ   →   pick the row where Δθ ≈ 0
empirical_v vs solver_v   →   the v ratio at that row is your slip ratio
```

You **must** match angles first. At the same distance, a higher launch angle
needs a higher v0 to reach the goal, so comparing wheel m/s at mismatched
angles is meaningless.

### Step 2 — compute the correction

At the row where angles agree:

```
correction = empirical_wheel_mps / solver_wheel_mps
new (η × wear) = current (η × wear) / correction
```

(Lower transfer ratio → solver commands higher wheel m/s.)

Example: at 1.95m the empirical LUT says 11.5 m/s, the solver says 10.70 m/s.
`correction = 11.5 / 10.70 = 1.075`. Current η × wear = 0.75 × 0.86 = 0.645.
New η × wear = 0.645 / 1.075 ≈ 0.60.

Realize as either `η = 0.70, wear = 0.86` or `η = 0.75, wear = 0.80`,
depending on which one changed (hardware vs ball).

### Step 3 — regenerate and check the whole table

Re-run the LUT generator. Now check the residuals at *every* row, not just
the one you tuned at. If the wheel m/s residuals are roughly flat across
distance, you're done. If they have shape, see below.

### Step 4 — angle bias

Compute `Δ = empirical_hood° − solver_hood°` for every row. If the values
cluster tightly around a single number, set `angle bias` to that number.
If they vary by more than ~3° across the table, **do not set the bias** to
the mean — it can't fix shape error, and forcing it will just smear the
error around. See below.

---

## What this tool can't do

The three sliders only correct **uniform** errors:

- η × wear: uniform multiplicative scaling of wheel m/s
- angle bias: uniform additive offset of hood°

They cannot correct *shape* mismatch between the solver's output and an
empirical curve. If you see residuals like:

```
1.4m:  Δθ = -3°
3.0m:  Δθ = +8°
4.6m:  Δθ = +10°
6.1m:  Δθ = +4°
```

...no single bias value will fix it. The shape mismatch comes from things
the tool intentionally doesn't model:

- **Robustness criterion.** `pickOptimal` chooses the centroid of the
  inscribed circle (equal weight on θ-margin and v-margin). Hand-tuned LUTs
  usually weight steeper entry angles more (better rim margin, better
  backspin grip). That's a *different* selection inside the same envelope.
- **No backspin / Magnus.** The trajectory is a passive sphere. Real shots
  have significant lift from backspin, especially with compliant wheels.
- **No mid-flight compression dynamics.** Soft balls deform on the wheel and
  fly differently than the rigid-sphere model assumes.

If you're trying to reproduce a hand-tuned LUT exactly: you won't. Expect
~2° / ~0.3 m/s residual as the floor for what this tool can match.

---

## When to trust the solver vs the empirical LUT

- **Trust empirical** for the current robot + current goal + current ball.
  If `loadLUTSoft` works, ship it.
- **Trust the solver** when something physical changes and you don't yet
  have empirical data: new hub height, new wheel radius, different ball
  mass, new field. The solver gives you a starting point you can refine
  with a few practice shots, instead of starting from scratch.

The shot-solver is most useful as a **what-if** tool, not as a replacement
for hand tuning.

---

## Quick reference

| Symptom | Likely fix |
|---|---|
| Solver wheel m/s uniformly low vs empirical | Lower `η × wear` |
| Solver wheel m/s uniformly high vs empirical | Raise `η × wear` |
| Hood° uniformly off by a constant | Set `angle bias` to the delta |
| Hood° residual has a shape (varies with distance) | Not fixable with sliders — change `pickOptimal` or accept the gap |
| Wheel m/s residual has a shape (varies with distance) | Match angles first; if shape persists, check Cd / launch geometry |
| Flat plateau in solver's hood angle across a distance band | Increase `i-th-n` (theta search steps) — likely grid resolution |
| New ball feels different mid-event | Re-measure `wear` only, leave `η` alone |
