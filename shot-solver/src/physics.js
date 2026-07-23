// 2D trajectory simulation. x is downrange distance from launcher pivot, y is height
// above field floor. Forces: gravity + quadratic aerodynamic drag.
//
//   m·dv/dt = -m·g·ŷ - ½·ρ·Cd·A·|v|·v
//
// Integrated with RK4 at fixed dt. Stops when the ball passes the user-defined
// stop_x (downrange cutoff) or drops below y=0.

const DEFAULT_DT = 0.002;
const MAX_TIME = 5.0;

function dragK(env) {
  if (!env.dragEnabled) return 0;
  const A = Math.PI * (env.diameter / 2) ** 2;
  return 0.5 * env.rho * env.cd * A / env.mass;
}

function deriv(vx, vy, k, g) {
  const speed = Math.hypot(vx, vy);
  return {
    ax: -k * speed * vx,
    ay: -g - k * speed * vy,
  };
}

// Single RK4 step. Returns new state.
function step(s, dt, k, g) {
  const k1 = deriv(s.vx, s.vy, k, g);
  const k2 = deriv(s.vx + 0.5 * dt * k1.ax, s.vy + 0.5 * dt * k1.ay, k, g);
  const k3 = deriv(s.vx + 0.5 * dt * k2.ax, s.vy + 0.5 * dt * k2.ay, k, g);
  const k4 = deriv(s.vx + dt * k3.ax, s.vy + dt * k3.ay, k, g);
  const ax = (k1.ax + 2 * k2.ax + 2 * k3.ax + k4.ax) / 6;
  const ay = (k1.ay + 2 * k2.ay + 2 * k3.ay + k4.ay) / 6;
  const vxAvg = s.vx + 0.5 * ax * dt;
  const vyAvg = s.vy + 0.5 * ay * dt;
  return {
    t: s.t + dt,
    x: s.x + vxAvg * dt,
    y: s.y + vyAvg * dt,
    vx: s.vx + ax * dt,
    vy: s.vy + ay * dt,
  };
}

/**
 * Simulate a launch.
 * @param {number} v0       launch speed (m/s)
 * @param {number} thetaRad hood angle above horizontal (rad)
 * @param {{x0:number, y0:number, dragEnabled:boolean, mass:number, diameter:number,
 *          rho:number, cd:number, g:number, dt?:number, stopX?:number}} env
 * @returns {{xs:number[], ys:number[], ts:number[], crossX:number|null, crossT:number|null,
 *           crossVy:number|null, apexY:number, apexX:number}}
 */
export function simulate(v0, thetaRad, env) {
  const k = dragK(env);
  const g = env.g;
  const dt = env.dt ?? DEFAULT_DT;
  const stopX = env.stopX ?? 50;

  let s = {
    t: 0,
    x: env.x0,
    y: env.y0,
    vx: v0 * Math.cos(thetaRad),
    vy: v0 * Math.sin(thetaRad),
  };
  const xs = [s.x];
  const ys = [s.y];
  const ts = [s.t];
  let apexY = s.y, apexX = s.x;

  while (s.t < MAX_TIME && s.x < stopX) {
    const next = step(s, dt, k, g);
    xs.push(next.x);
    ys.push(next.y);
    ts.push(next.t);
    if (next.y > apexY) { apexY = next.y; apexX = next.x; }
    if (next.y < 0 && next.vy < 0) break;
    s = next;
  }
  return { xs, ys, ts, apexY, apexX, final: s };
}

/**
 * Simulate until the trajectory crosses goalY while descending, or until it goes past
 * stopX / falls to ground. Linearly interpolates the crossing point.
 *
 * @returns {{x:number|null, t:number|null, vx:number|null, vy:number|null}}
 */
export function simulateToGoalY(v0, thetaRad, env, goalY) {
  const k = dragK(env);
  const g = env.g;
  const dt = env.dt ?? DEFAULT_DT;
  const stopX = env.stopX ?? 50;

  let s = {
    t: 0,
    x: env.x0,
    y: env.y0,
    vx: v0 * Math.cos(thetaRad),
    vy: v0 * Math.sin(thetaRad),
  };
  let reachedApex = s.vy <= 0;

  while (s.t < MAX_TIME && s.x < stopX) {
    const next = step(s, dt, k, g);
    if (!reachedApex && next.vy <= 0) reachedApex = true;
    // Descending and crossing goalY
    if (reachedApex && next.vy < 0 && s.y >= goalY && next.y < goalY) {
      const f = (s.y - goalY) / (s.y - next.y);
      return {
        x: s.x + f * (next.x - s.x),
        t: s.t + f * (next.t - s.t),
        vx: s.vx + f * (next.vx - s.vx),
        vy: s.vy + f * (next.vy - s.vy),
      };
    }
    if (next.y < 0 && next.vy < 0) break;
    s = next;
  }
  return { x: null, t: null, vx: null, vy: null };
}

/**
 * Height of trajectory at a given downrange x, on the descending leg. Used for funnel
 * clearance checks (does the ball clear the close rim of the funnel wall?).
 */
export function heightAtX(v0, thetaRad, env, targetX) {
  const k = dragK(env);
  const g = env.g;
  const dt = env.dt ?? DEFAULT_DT;

  let s = {
    t: 0,
    x: env.x0,
    y: env.y0,
    vx: v0 * Math.cos(thetaRad),
    vy: v0 * Math.sin(thetaRad),
  };
  while (s.t < MAX_TIME) {
    const next = step(s, dt, k, g);
    if (s.x <= targetX && next.x >= targetX) {
      const f = (targetX - s.x) / (next.x - s.x);
      return s.y + f * (next.y - s.y);
    }
    if (next.y < 0 && next.vy < 0) return null;
    if (next.x > targetX + 1) return null;
    s = next;
  }
  return null;
}
