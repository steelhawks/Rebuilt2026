# Tuning the shot-solver

This doc covers the four sliders that don't come from physics first principles:

- `η` (slip efficiency) — wheel-to-ball energy transfer
- `wear` (ball condition) — degrades η over the event
- `angle bias` (hood ↔ ball-launch offset) — measurement, set from slo-mo
- `shot placement bias` (front rim ↔ back rim) — strategic choice

If your physical constants (mass, diameter, Cd, ρ, g, hub geometry, launch
height) are correct, these four knobs are the only things you should be
touching to make the solver match the real robot.

---

## The core idea

The solver gives you a valid (θ, v) **envelope** for any distance — a band
inside which any shot makes the goal. Inside that envelope there's a single
strategic choice (where in the band to live) and two measurements that bake
all unmodeled losses (slip, ball compression, hood encoder offset, drag bias)
into scalars.

The whole workflow comes down to three knobs:

- **shot placement bias**: where in the valid envelope to aim — front rim,
  center, or back rim. Strategic choice, set once.
- **angle bias**: hood encoder vs actual ball-launch angle. Measure with
  **slo-mo video**.
- **η × wear** (the slip ratio): wheel surface m/s vs ball exit m/s. Tune by
  **shooting the robot** until balls go in.

Hightide's 2025 approach used the centroid (placement = 0.5) and the two
measurements:

> "The goal is not to eliminate empirical tuning — it's to bake the magic
> black-box numbers into a single value we can tune."

We push one step further with **back-rim biased placement** (placement ≈ 0.75)
because real errors aren't symmetric — see the next section.

---

## Why back-rim bias

The valid envelope is a band of (θ, v) pairs that all make the goal. The
centroid maximizes symmetric tolerance — equal slack in every direction.
That's the right pick if errors are symmetric. **They aren't.**

Almost every real-world error pulls the ball **short**:

- Drag is always there and unmodeled drag underestimates the slowdown.
- Worn balls have lower transfer ratio → lower exit velocity → land short.
- Voltage sag on a heavy command spike → flywheel undershoots → lands short.
- Backspin generates lift but backspin itself varies; less spin than expected
  → lands short.
- Vision range estimate at oblique tag angles tends to read further than
  reality → solver thinks it needs less speed → ball lands short.

Aiming at the centroid: those errors push you toward the front rim or short
of the goal entirely.

Aiming at the back rim: the *same errors* push you toward the center.

The placement bias is one slider that does this:

- `0.5` = centroid (Hightide / classic robustness)
- `0.75` = back-rim biased (default — converts short-bias errors into makes)
- `1.0 − safety` = aim at the back-rim edge (most aggressive — minimal headroom)

Side effect: the back-rim biased shot is also slightly faster to spin up at
some distances, because v at high bias can be lower than v_far on angles
where the envelope is asymmetric. This is a small bonus, not the point.

## The workflow (do this in order)

### Step 1 — generate the LUT from the solver

Set `angle bias = 0`, `placement bias ≈ 0.75`, and `η × wear` to a reasonable
starting guess (0.6–0.7 for a typical compliant-wheel shooter). Run the LUT
generator. Paste the output into `ShooterStructure.java` as `loadLUTSolved`.

**Do not try to make this match `loadLUTSoft` row-by-row.** Although: with
placement bias around 0.75, the generated LUT will look *much* more like
`loadLUTSoft` than the centroid version did, because experienced hand-tuners
implicitly converge on back-rim-biased shots for the same asymmetric-error
reason. If the bias LUT and the soft LUT agree within ~2° / ~0.3 m/s, that's
strong evidence the physics is right and the bias matches the team's intuition.

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
| Setting up a new shooter, no LUT exists | Generate from solver with `placement bias ≈ 0.75`, then Steps 2–3 |
| Balls consistently short | Lower `η × wear` |
| Balls consistently long | Raise `η × wear` |
| Balls clear the goal cleanly half the time, rim-out front the other half | Raise `placement bias` — you're at centroid, errors are pulling you toward front rim |
| Balls fly long when fresh, short when worn | Re-tune `wear` (Step 4) |
| Balls go short at near range, long at far (or vice versa) | Slip varies with speed — go to "When one scalar isn't enough" |
| Hood reads X° but slo-mo shows ball leaving at Y° | Set `angle bias = Y − X` |
| Trying to match an existing empirical LUT row-by-row | If hand-tuned LUT looks faster + steeper than centroid LUT, set `placement bias ≈ 0.75` and regenerate — they should agree |
| New ball stock mid-event, robot starts missing | Re-tune `wear` only (Step 3 with the new ball) |
| Flat plateau in solver hood angle across distance | Bump `i-th-n` (theta search steps) — grid resolution |
