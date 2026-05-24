// Shot envelope + robust-optimal selection.
//
// For each launch angle θ in the search grid, find the two launch speeds that
// (a) just clip the close rim and (b) just reach the far rim. Those two curves
// in (θ, v) space bound the "valid region": any (θ, v) inside makes the goal.
//
// The optimal shot is the point inside that region with the largest distance
// (in normalized angle/speed units) to the nearest boundary — i.e. the centre
// of the inscribed circle. That maximises robustness to combined angle + speed
// error, which is what we actually care about on the robot.
//
// Approach is the 1690-2022 method, generalised to arbitrary force model: the
// solver doesn't know whether drag is on or off, it just calls simulateToGoalY.

import { simulateToGoalY } from "./physics.js";

// Search for v that produces crossing_x = targetX at a fixed angle θ, by bisection.
// crossing_x is monotonically increasing in v (in the relevant range) so this is well-posed.
function bisectVForCrossing(thetaRad, env, targetX, goalY, vLo, vHi, depth = 32) {
  const xAt = (v) => {
    const r = simulateToGoalY(v, thetaRad, env, goalY);
    return r.x;
  };
  let xLo = xAt(vLo);
  let xHi = xAt(vHi);
  // Expand bounds if needed (search hasn't bracketed yet).
  let tries = 0;
  while ((xHi == null || xHi < targetX) && vHi < 60 && tries < 8) {
    vHi *= 1.5;
    xHi = xAt(vHi);
    tries++;
  }
  if (xHi == null) return null;
  if (xLo != null && xLo > targetX) return null; // already overshoots at vLo
  for (let i = 0; i < depth; i++) {
    const vm = 0.5 * (vLo + vHi);
    const xm = xAt(vm);
    if (xm == null) {
      vLo = vm;
      continue;
    }
    if (xm < targetX) vLo = vm; else vHi = vm;
    if (vHi - vLo < 1e-4) break;
  }
  const vFinal = 0.5 * (vLo + vHi);
  // Validate the converged v actually reaches targetX. If the angle physically
  // cannot place a shot at targetX (e.g. trajectory apex is too low), bisection
  // will converge to whatever extreme it can reach — that's a false positive.
  const xFinal = bisectXAt(thetaRad, env, goalY, vFinal);
  if (xFinal == null || Math.abs(xFinal - targetX) > 0.05) return null;
  return vFinal;
}

function bisectXAt(thetaRad, env, goalY, v) {
  const r = simulateToGoalY(v, thetaRad, env, goalY);
  return r.x;
}

/**
 * Build the close-rim and far-rim speed curves across the angle grid.
 *
 * @param {{distance:number, hubZ:number, hubR:number, funnelH:number, clearance:number,
 *          launchZ:number, launchX:number}} geom
 * @param {{thetaMinDeg:number, thetaMaxDeg:number, thetaSteps:number, vCap:number}} search
 * @param {object} env physics environment (drag, mass, etc.)
 */
export function solveEnvelope(geom, search, env) {
  // Goal rim height. If the user is modeling a funnel wall that extends above
  // the mouth, raise the crossing surface by funnelH + clearance. With funnelH=0
  // and clearance=0 this collapses to the 1690 flat-opening model used in the
  // reference images.
  const goalY = geom.hubZ + geom.funnelH + geom.clearance;
  const mouthY = geom.hubZ;  // drawn separately for context
  // close rim is the near edge of the goal opening (in launcher-relative downrange x).
  const xClose = geom.distance - geom.launchX - geom.hubR;
  const xFar = geom.distance - geom.launchX + geom.hubR;

  // stopX must be generous: during bisection we probe high speeds whose trajectories
  // cross goalY far past the goal. Tight stopX makes simulateToGoalY return null
  // and the bisection mistakenly thinks the angle is infeasible.
  const eEnv = { ...env, x0: 0, y0: geom.launchZ, stopX: 50 };

  const thetas = [];
  const dTh = (search.thetaMaxDeg - search.thetaMinDeg) / (search.thetaSteps - 1);
  for (let i = 0; i < search.thetaSteps; i++) {
    thetas.push((search.thetaMinDeg + i * dTh) * Math.PI / 180);
  }

  const vCloseArr = [];
  const vFarArr = [];
  const validMask = [];

  for (const th of thetas) {
    const vC = bisectVForCrossing(th, eEnv, xClose, goalY, 1.0, search.vCap);
    const vF = bisectVForCrossing(th, eEnv, xFar, goalY, 1.0, search.vCap);
    const valid = vC != null && vF != null && vF > vC;
    vCloseArr.push(vC);
    vFarArr.push(vF);
    validMask.push(valid);
  }

  return {
    thetas,
    vClose: vCloseArr,
    vFar: vFarArr,
    valid: validMask,
    geom: { xClose, xFar, goalY, mouthY },
  };
}

/**
 * Pick the most-robust shot inside the envelope.
 *
 * Robustness metric: at each candidate (θ, v) inside the band, measure the four
 * axis-aligned distances to the nearest boundary — increasing v, decreasing v,
 * increasing θ (until v leaves the band), decreasing θ. Normalize by overall
 * envelope span and take the geometric mean of the (smaller of each pair).
 * Geometric mean rewards points whose smaller dimension is non-trivial, so it
 * doesn't reward "wide v band right at the search-range corner."
 */
export function pickOptimal(env) {
  const { thetas, vClose, vFar, valid } = env;
  let thLo = Infinity, thHi = -Infinity, vLo = Infinity, vHi = -Infinity;
  for (let i = 0; i < thetas.length; i++) {
    if (!valid[i]) continue;
    thLo = Math.min(thLo, thetas[i]);
    thHi = Math.max(thHi, thetas[i]);
    vLo = Math.min(vLo, vClose[i]);
    vHi = Math.max(vHi, vFar[i]);
  }
  if (!isFinite(thLo)) return null;
  const thSpan = Math.max(thHi - thLo, 1e-6);
  const vSpan = Math.max(vHi - vLo, 1e-6);

  function angleRangeFor(v, thStar) {
    let iStar = 0, best = Infinity;
    for (let i = 0; i < thetas.length; i++) {
      const d = Math.abs(thetas[i] - thStar);
      if (d < best) { best = d; iStar = i; }
    }
    let iLo = iStar;
    while (iLo > 0) {
      const a = vClose[iLo - 1], b = vFar[iLo - 1];
      if (!valid[iLo - 1] || a == null || b == null || v < a || v > b) break;
      iLo--;
    }
    let iHi = iStar;
    while (iHi < thetas.length - 1) {
      const a = vClose[iHi + 1], b = vFar[iHi + 1];
      if (!valid[iHi + 1] || a == null || b == null || v < a || v > b) break;
      iHi++;
    }
    return { low: thStar - thetas[iLo], high: thetas[iHi] - thStar };
  }

  let best = null;
  const N_V = 24;
  for (let i = 0; i < thetas.length; i++) {
    if (!valid[i]) continue;
    const th = thetas[i];
    const vC = vClose[i];
    const vF = vFar[i];
    for (let j = 1; j < N_V - 1; j++) {
      const v = vC + (vF - vC) * (j / (N_V - 1));
      const ar = angleRangeFor(v, th);
      const sLo = (v - vC) / vSpan;
      const sHi = (vF - v) / vSpan;
      const aLo = ar.low / thSpan;
      const aHi = ar.high / thSpan;
      // Score = geometric mean of (min speed margin) and (min angle margin).
      // A point at the search-range edge has aLo or aHi = 0, killing the score.
      const sMin = Math.min(sLo, sHi);
      const aMin = Math.min(aLo, aHi);
      const score = Math.sqrt(sMin * aMin);
      if (best == null || score > best.score) {
        best = { thetaRad: th, v0: v, score };
      }
    }
  }
  return best;
}

/**
 * Compute tolerances at the chosen optimum:
 *   speed_low  = v* - v_close(θ*)
 *   speed_high = v_far(θ*) - v*
 *   angle_low/high — sweep angles outwards from θ* while v* stays inside the band.
 */
export function tolerances(env, optimal) {
  const { thetas, vClose, vFar } = env;
  // Find nearest angle index to θ*.
  let bestI = 0, bestD = Infinity;
  for (let i = 0; i < thetas.length; i++) {
    const d = Math.abs(thetas[i] - optimal.thetaRad);
    if (d < bestD) { bestD = d; bestI = i; }
  }
  // Linear-interp the v_close/v_far at θ* using neighbours.
  function interpAt(arr, th) {
    for (let i = 0; i < thetas.length - 1; i++) {
      if (th >= thetas[i] && th <= thetas[i + 1]) {
        const f = (th - thetas[i]) / (thetas[i + 1] - thetas[i]);
        const a = arr[i], b = arr[i + 1];
        if (a == null || b == null) return null;
        return a + f * (b - a);
      }
    }
    return null;
  }
  const vC = interpAt(vClose, optimal.thetaRad);
  const vF = interpAt(vFar, optimal.thetaRad);
  const speedLow = vC != null ? optimal.v0 - vC : null;
  const speedHigh = vF != null ? vF - optimal.v0 : null;

  // Angle bounds: sweep outward from bestI while v* is inside [v_close, v_far] at that θ.
  let iLo = bestI;
  while (iLo > 0) {
    const a = vClose[iLo - 1], b = vFar[iLo - 1];
    if (a == null || b == null || optimal.v0 < a || optimal.v0 > b) break;
    iLo--;
  }
  let iHi = bestI;
  while (iHi < thetas.length - 1) {
    const a = vClose[iHi + 1], b = vFar[iHi + 1];
    if (a == null || b == null || optimal.v0 < a || optimal.v0 > b) break;
    iHi++;
  }
  const angleLow = optimal.thetaRad - thetas[iLo];
  const angleHigh = thetas[iHi] - optimal.thetaRad;

  return { speedLow, speedHigh, angleLow, angleHigh };
}
