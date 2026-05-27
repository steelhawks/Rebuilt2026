# Tuning the shot-solver

This doc covers the three sliders that don't come from physics first principles:

- `η` (slip efficiency)
- `wear` (ball condition)
- `angle bias` (hood ↔ ball-launch offset)

If your physical constants (mass, diameter, Cd, ρ, g, hub geometry, launch
height) are correct, these three knobs are the only things you should be
touching to make the solver match the real robot.

---

## The core idea

The solver gives you a physics-optimal (θ, v) for any distance. Real shooters
have unmodeled losses — wheel slip, ball compression, backspin, hood
mechanical offset — that no amount of constants tweaking will perfectly
capture. Instead of trying to model every loss, **collapse all of them into
two scalars** that you tune from physical measurement:

- **angle bias**: hood encoder reads one angle, ball actually leaves at another.
  Measure with **slo-mo video**.
- **η × wear** (the slip ratio): wheel surface m/s vs ball exit m/s. Tune by
  **shooting the robot** until balls go in.

This is the approach Hightide used in 2025. Their summary, paraphrased:

> "The goal is not to eliminate empirical tuning — it's to bake the magic
> black-box numbers into a single value we can tune."

Two scalars. That's the whole tune.

---

## The workflow (do this in order)

### Step 1 — generate the LUT from the solver

Set `angle bias = 0` and `η × wear` to a reasonable starting guess (0.6–0.7
for a typical compliant-wheel shooter). Run the LUT generator. Paste the
output into `ShooterStructure.java` as `loadLUTSolved`.

**Do not try to make this match `loadLUTSoft` row-by-row.** They are different
optimization problems. `loadLUTSoft` is whatever (θ, v) the drivers settled on
inside the valid envelope; the solver picks the robustness-optimum inside the
same envelope. Both make shots. Their rows will not agree, and they don't
need to.

### Step 2 — tune `angle bias` from slo-mo

**This is the most important step. Do it first, before touching slip.**

1. Set up a phone or camera at 120 fps or higher, side-on to the shooter.
   Frame the shooter so the ball's first ~30 cm of flight is visible.
2. Shoot the robot at any single distance with a known hood command. Record.
3. Step through the slo-mo frame by frame. Find the frame the ball leaves the
   shooter. Measure the ball's velocity vector angle from horizontal — two
   ways that both work:
   - Draw a line between the ball center in two consecutive frames; measure
     that line's angle.
   - Use a video annotation tool (Tracker, Kinovea, even a protractor overlay
     in the iOS Photos editor) to read the angle directly.
4. Compare to the **hood encoder angle** that was commanded at that shot.
5. `angle bias = measured_ball_angle - hood_command_angle`. Usually negative
   (hood reads higher than the ball launches, because soft balls compress and
   flatten on exit).
6. Plug it into the `angle bias` slider. Regenerate the LUT.

Hightide reports this offset is **near-constant across all distances**, so one
shot is enough. If you want to be thorough, repeat at two distances (near and
far) and check the value is the same — if it isn't, your slo-mo measurement
has more noise than the underlying physics.

### Step 3 — tune `η × wear` by shooting the robot

With the angle bias set, the hood is now pointing where the ball will actually
launch. The only remaining variable is whether the wheel speed gets enough ball
exit velocity.

1. Drive to a known distance. Shoot.
2. If balls are **short**, raise the commanded wheel speed → that means the
   ball needs more energy than the solver thinks → **lower `η × wear`**
   (so `v_wheel = v_ball / (η × wear)` goes up).
3. If balls are **long**, **raise `η × wear`**.
4. Single scalar. Adjust until balls go in. Regenerate the LUT.

Hightide: *"you just 'fudge' the constant until the balls start going in the
goal, and poof… suddenly all shots make it."*

The reason this works as a single scalar is that the slip loss is roughly
proportional to wheel speed across the working range — so one multiplier
adjusts the entire LUT uniformly. If you find that one end of the LUT shoots
short while the other shoots long after the bias is set, see "When one scalar
isn't enough" below.

### Step 4 — re-tune `wear` when balls change

Same procedure as Step 3, but **only move `wear`**. Leave `η` alone — that
captures the shooter, which didn't change.

If you only ever have one ball type, you don't need the split. Leave
`wear = 1.0` and put everything in `η`.

---

## What each knob actually is

### η × wear (the transfer ratio)

```
v_ball = (η × wear) × v_wheel
```

`v_wheel` is wheel surface velocity = `ω · r` = `motor_rev_per_sec × 2π ×
wheel_radius`.

The split:
- `η` = shooter property (wheel hardness, compression preload, dwell). Fixed
  until you change hardware.
- `wear` = ball property (1.0 fresh, ~0.85 soft worn). Re-tune per event.

Mathematically they're the same knob — only their product is observable from
a single shot. To separate, you need a reference: measure with a known-good
ball (defines η, wear=1.0), then measure again later (wear = today's ratio / η).

### angle bias (degrees)

A uniform offset added to the physics-optimal launch angle to produce the hood
command:

```
hood_command_deg = physics_optimal_deg + angle_bias
```

Captures the difference between what the hood encoder reads and the actual ball
launch direction. Sources: ball compression flattening the exit angle, hood
flex under load, encoder zeroing offset, geometry between hood pivot and ball
contact patch.

**Constant offset only.** It is not a function of distance.

---

## Optional — measuring η directly with a chrono

You don't need this if Steps 2 and 3 above worked. But if you have a
chronograph or radar gun and want to set `η` from first measurement instead
of fudging:

1. Pick a known-good ball.
2. Command a wheel surface velocity `v_wheel` you can read off the dashboard.
3. Measure ball exit `v_ball` right at the shooter exit.
4. `η = v_ball / v_wheel`. Set `wear = 1.0`.

Sanity check: typical FRC compliant-wheel shooter lands in **0.5–0.8**. Outside
that range you measured the wrong thing — re-check that `v_wheel` is surface
velocity (not motor RPS) and that `v_ball` is right at exit (not after meters
of drag).

---

## When one scalar isn't enough

Hightide flagged this as a real failure mode:

> "one revision was not linear and we had a small lookup map for rps→mps based
> on distance"

If after Steps 2 and 3 you find balls go in at mid-range but consistently
miss at one end (and you've checked that the angle bias is right across
distances via slo-mo), the slip ratio itself varies with shot speed.
Compression dwell, contact geometry under load, or wheel surface response
can all cause this.

Fix: replace the scalar `η × wear` with a small interpolated map keyed by
distance (or ball exit speed). This is still much simpler than a full
`(distance → θ, v)` empirical LUT — you're tuning one curve, not three.

The hook is in `slip.js`: `transfer(slip)` currently returns the scalar
product. Swap it for a small NavigableMap lookup if you need this.

---

## What this tool can't do

Even with perfect tuning, the solver will not reproduce a hand-tuned LUT
exactly. The two answer different questions:

- **Solver**: "what (θ, v) is the robustness-optimum inside the valid envelope?"
- **Hand-tuned LUT**: "what (θ, v) did the drivers settle on inside that same
  envelope?"

Both make shots. They will diverge by a few degrees and a few tenths of m/s
because:

- The robustness metric in `pickOptimal` (solver.js:174) weights θ- and
  v-margin equally. Hand tuners often weight steeper entry more (rim margin,
  backspin grip).
- The physics is a passive sphere — no backspin lift / Magnus, no mid-flight
  compression dynamics.

**If you have a hand-tuned LUT that scores reliably, ship it.** The solver's
value is for *new geometry* (new hub height, new wheel radius, different ball
mass, new field) where you don't yet have empirical data. It gets you to
"balls roughly going in" in two scalar tunes instead of weeks of hand tuning.

---

## Quick reference

| Symptom | Fix |
|---|---|
| Setting up a new shooter, no LUT exists | Generate from solver, then Steps 2–3 |
| Balls consistently short | Lower `η × wear` |
| Balls consistently long | Raise `η × wear` |
| Balls go short at near range, long at far (or vice versa) | Slip varies with speed — go to "When one scalar isn't enough" |
| Hood reads X° but slo-mo shows ball leaving at Y° | Set `angle bias = Y − X` |
| Trying to match an existing empirical LUT row-by-row | Don't. They won't match. Tune from the real robot. |
| New ball stock mid-event, robot starts missing | Re-tune `wear` only (Step 3 with the new ball) |
| Flat plateau in solver hood angle across distance | Bump `i-th-n` (theta search steps) — grid resolution |
