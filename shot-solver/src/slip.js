// Wheel-slip / ball-condition transfer model.
//
// The flywheel surface velocity v_w = ω·r is the upper bound on the ball's exit
// velocity. Real shooters lose energy to: ball compression, ball spin pickup,
// short contact dwell, surface wear. We collapse all of that into two scalars:
//
//   η_slip   — geometric/mechanical slip (set per shooter revision)
//   η_wear   — ball condition multiplier (1.0 fresh hard, ~0.85 soft worn)
//
//   v_ball = η_slip · η_wear · v_wheel
//
// This matches the team's empirical observation: as balls soften, every entry in
// the LUT shifts up by a near-constant factor — one knob (η_wear) is enough to
// re-tune the whole map. See ShooterStructure.loadLUTHard vs loadLUTSoft: the
// commanded flywheel m/s rises ~5% across the board between the two LUTs, which
// corresponds to η_wear dropping by ~5%.

/** Effective transfer ratio v_ball / v_wheel. */
export function transfer(slip) {
  return slip.eta * slip.wear;
}

/** Wheel surface velocity (m/s) needed to produce a desired ball velocity. */
export function wheelMpsForBall(ballMps, slip) {
  const t = transfer(slip);
  if (t <= 0) return Infinity;
  return ballMps / t;
}

/** Flywheel rotational speed (rev/s) for a desired ball velocity. */
export function wheelRpsForBall(ballMps, slip, wheelRadius) {
  const surface = wheelMpsForBall(ballMps, slip);
  return surface / (2 * Math.PI * wheelRadius);
}

/** Ball exit velocity (m/s) when commanding a flywheel surface velocity. */
export function ballMpsFromWheel(wheelMps, slip) {
  return wheelMps * transfer(slip);
}
