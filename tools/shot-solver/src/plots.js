// Plotly rendering — matches the dark navy style of the reference images.
// Two charts:
//   1. Trajectory envelope:  family of (close-rim, far-rim) trajectories per angle,
//      optimal trajectory thick in blue, hub + funnel obstacle drawn.
//   2. Speed × angle valid region: v_close(θ) and v_far(θ) curves with the band
//      shaded, optimal star, speed and angle tolerance indicators.

import { simulate } from "./physics.js";

// Truncate a trajectory at the goal-rim crossing (descending leg). Without this
// the simulated arc continues through the goal and into the floor, which makes
// the close/far family look like a confusing fan past the hub.
function truncAtGoal(xs, ys, goalY) {
  if (xs.length < 2) return { xs, ys };
  let reachedApex = false;
  for (let i = 1; i < ys.length; i++) {
    if (!reachedApex && ys[i] < ys[i - 1]) reachedApex = true;
    if (reachedApex && ys[i - 1] >= goalY && ys[i] < goalY) {
      const f = (ys[i - 1] - goalY) / (ys[i - 1] - ys[i]);
      const xCross = xs[i - 1] + f * (xs[i] - xs[i - 1]);
      return { xs: [...xs.slice(0, i), xCross], ys: [...ys.slice(0, i), goalY] };
    }
  }
  return { xs, ys };
}

const COLOR = {
  bg: "#0e1a36",
  grid: "#243a73",
  text: "#e7ecf7",
  textDim: "#9eb0d6",
  close: "#ff5252",
  far: "#4caf50",
  opt: "#4f9cff",
  speed: "#ff9a3c",
  angle: "#b48cff",
  warn: "#ffcf4a",
  fill: "rgba(76,175,80,0.18)",
};

const BASE_LAYOUT = {
  paper_bgcolor: COLOR.bg,
  plot_bgcolor: COLOR.bg,
  font: { color: COLOR.text, family: "ui-sans-serif, system-ui, Inter", size: 12 },
  margin: { l: 50, r: 16, t: 12, b: 42 },
  showlegend: false,
  hovermode: "closest",
};

const AXIS = {
  gridcolor: COLOR.grid,
  zerolinecolor: COLOR.grid,
  linecolor: COLOR.grid,
  tickcolor: COLOR.grid,
  tickfont: { color: COLOR.textDim },
  titlefont: { color: COLOR.textDim },
};

const CONFIG = {
  responsive: true,
  displaylogo: false,
  modeBarButtonsToRemove: ["lasso2d", "select2d"],
};

/** Trajectory envelope plot. */
export function drawTrajectory(divId, env, geom, optimal, showAll) {
  const traces = [];

  // Per-angle close-rim trajectories (red) and far-rim trajectories (green).
  if (showAll) {
    for (let i = 0; i < env.thetas.length; i++) {
      if (!env.valid[i]) continue;
      const th = env.thetas[i];
      if (env.vClose[i] != null) {
        const t = simulate(env.vClose[i], th, {
          x0: 0, y0: geom.launchZ, dragEnabled: geom.dragEnabled,
          mass: geom.mass, diameter: geom.diameter, rho: geom.rho,
          cd: geom.cd, g: geom.g, stopX: env.geom.xFar + 0.6,
        });
        const cut = truncAtGoal(t.xs, t.ys, env.geom.goalY);
        traces.push({
          x: cut.xs, y: cut.ys, mode: "lines", type: "scatter",
          line: { color: COLOR.close, width: 1 }, opacity: 0.55, hoverinfo: "skip",
        });
      }
      if (env.vFar[i] != null) {
        const t = simulate(env.vFar[i], th, {
          x0: 0, y0: geom.launchZ, dragEnabled: geom.dragEnabled,
          mass: geom.mass, diameter: geom.diameter, rho: geom.rho,
          cd: geom.cd, g: geom.g, stopX: env.geom.xFar + 0.6,
        });
        const cut = truncAtGoal(t.xs, t.ys, env.geom.goalY);
        traces.push({
          x: cut.xs, y: cut.ys, mode: "lines", type: "scatter",
          line: { color: COLOR.far, width: 1 }, opacity: 0.55, hoverinfo: "skip",
        });
      }
    }
  }

  // Optimal trajectory (thick blue), also truncated at the goal.
  if (optimal) {
    const t = simulate(optimal.v0, optimal.thetaRad, {
      x0: 0, y0: geom.launchZ, dragEnabled: geom.dragEnabled,
      mass: geom.mass, diameter: geom.diameter, rho: geom.rho,
      cd: geom.cd, g: geom.g, stopX: env.geom.xFar + 0.6,
    });
    const cut = truncAtGoal(t.xs, t.ys, env.geom.goalY);
    traces.push({
      x: cut.xs, y: cut.ys, mode: "lines", type: "scatter",
      line: { color: COLOR.opt, width: 4 },
      name: "Optimal",
      hovertemplate: `x %{x:.2f} m<br>y %{y:.2f} m<extra></extra>`,
    });
  }

  const xMax = Math.max(env.geom.xFar + 1.2, geom.distance + 1.5);
  const yMax = Math.max(...traces.flatMap(t => t.y ?? [0]), geom.hubZ + 1.2, 3);

  // Hub markers — red vertical at close rim, green horizontal across the goal mouth.
  const shapes = [
    // dashed horizontal at goal Y
    {
      type: "line", x0: 0, x1: xMax, y0: env.geom.goalY, y1: env.geom.goalY,
      line: { color: COLOR.close, width: 1, dash: "dash" }, opacity: 0.5,
    },
    // close rim vertical (red)
    {
      type: "line", x0: env.geom.xClose, x1: env.geom.xClose, y0: 0, y1: env.geom.goalY,
      line: { color: COLOR.close, width: 3 },
    },
    // far rim vertical (faint)
    {
      type: "line", x0: env.geom.xFar, x1: env.geom.xFar, y0: 0, y1: env.geom.goalY,
      line: { color: COLOR.close, width: 1, dash: "dot" }, opacity: 0.5,
    },
    // goal mouth (green horizontal between rims)
    {
      type: "rect",
      x0: env.geom.xClose, x1: env.geom.xFar,
      y0: env.geom.goalY - 0.04, y1: env.geom.goalY + 0.04,
      fillcolor: COLOR.far, line: { width: 0 }, opacity: 0.85,
    },
  ];
  // If a funnel wall raises the crossing surface above the bare mouth, draw the
  // wall outline as a dotted yellow box from mouth to rim height.
  if (env.geom.goalY > env.geom.mouthY + 1e-3) {
    shapes.push({
      type: "rect",
      x0: env.geom.xClose, x1: env.geom.xFar,
      y0: env.geom.mouthY, y1: env.geom.goalY,
      line: { color: COLOR.warn, width: 1, dash: "dot" },
      fillcolor: "rgba(255,207,74,0.04)",
    });
  }
  // Close-rim "ball" marker
  const markers = [{
    x: [env.geom.xClose], y: [env.geom.goalY], type: "scatter", mode: "markers",
    marker: { color: "#ffffff", size: 12, line: { color: COLOR.close, width: 3 } },
    hoverinfo: "skip",
  }];

  Plotly.react(divId, [...traces, ...markers], {
    ...BASE_LAYOUT,
    xaxis: { ...AXIS, title: "Distance (m)", range: [0, xMax], dtick: 1 },
    yaxis: { ...AXIS, title: "Height (m)", range: [0, yMax], scaleanchor: "x", scaleratio: 1 },
    shapes,
  }, CONFIG);
}

/** Speed × angle envelope (the U-shape plot from reference image 2). */
export function drawEnvelope(divId, env, optimal, tol) {
  const thDeg = env.thetas.map(t => t * 180 / Math.PI);

  // Build paired arrays for filled band (only across valid runs).
  const xFill = [];
  const yFill = [];
  // Walk forward along close curve, back along far curve.
  for (let i = 0; i < thDeg.length; i++) {
    if (env.valid[i]) { xFill.push(thDeg[i]); yFill.push(env.vClose[i]); }
  }
  for (let i = thDeg.length - 1; i >= 0; i--) {
    if (env.valid[i]) { xFill.push(thDeg[i]); yFill.push(env.vFar[i]); }
  }
  if (xFill.length > 0) {
    xFill.push(xFill[0]); yFill.push(yFill[0]);
  }

  const traces = [
    {
      x: xFill, y: yFill, type: "scatter", mode: "lines",
      fill: "toself", fillcolor: COLOR.fill, line: { width: 0 },
      hoverinfo: "skip",
    },
    {
      x: thDeg, y: env.vClose, type: "scatter", mode: "lines+markers",
      line: { color: COLOR.close, width: 2 },
      marker: { color: COLOR.close, size: 4 },
      name: "Close rim",
      hovertemplate: "θ %{x:.1f}°<br>v %{y:.2f} m/s<extra>close</extra>",
    },
    {
      x: thDeg, y: env.vFar, type: "scatter", mode: "lines+markers",
      line: { color: COLOR.far, width: 2 },
      marker: { color: COLOR.far, size: 4 },
      name: "Far rim",
      hovertemplate: "θ %{x:.1f}°<br>v %{y:.2f} m/s<extra>far</extra>",
    },
  ];

  const shapes = [];
  if (optimal) {
    const thOptDeg = optimal.thetaRad * 180 / Math.PI;
    // Speed tolerance: vertical orange bar from v_close to v_far at θ*
    if (tol && tol.speedLow != null && tol.speedHigh != null) {
      shapes.push({
        type: "line",
        x0: thOptDeg, x1: thOptDeg,
        y0: optimal.v0 - tol.speedLow, y1: optimal.v0 + tol.speedHigh,
        line: { color: COLOR.speed, width: 3 },
      });
    }
    // Angle tolerance: horizontal purple bar across the band at v*
    if (tol && tol.angleLow != null && tol.angleHigh != null) {
      shapes.push({
        type: "line",
        x0: thOptDeg - tol.angleLow * 180 / Math.PI,
        x1: thOptDeg + tol.angleHigh * 180 / Math.PI,
        y0: optimal.v0, y1: optimal.v0,
        line: { color: COLOR.angle, width: 3 },
      });
    }
    // Ellipse outline (visual)
    if (tol && tol.speedLow != null) {
      const dxDeg = (tol.angleLow + tol.angleHigh) * 90 / Math.PI; // half-width in deg
      const dy = 0.5 * ((tol.speedLow ?? 0) + (tol.speedHigh ?? 0));
      shapes.push({
        type: "circle",
        xref: "x", yref: "y",
        x0: thOptDeg - dxDeg, x1: thOptDeg + dxDeg,
        y0: optimal.v0 - dy, y1: optimal.v0 + dy,
        line: { color: COLOR.warn, width: 2 },
        fillcolor: "rgba(255,207,74,0.05)",
      });
    }
    traces.push({
      x: [thOptDeg], y: [optimal.v0],
      type: "scatter", mode: "markers",
      marker: { color: COLOR.opt, size: 16, symbol: "star",
                line: { color: "#ffffff", width: 1 } },
      name: "Optimal",
      hovertemplate: "θ* %{x:.1f}°<br>v* %{y:.2f} m/s<extra></extra>",
    });
  }

  // Auto-scale ranges
  const validV = env.vClose.concat(env.vFar).filter(v => v != null);
  const vLo = validV.length ? Math.min(...validV) - 0.3 : 0;
  const vHi = validV.length ? Math.max(...validV) + 0.3 : 20;

  Plotly.react(divId, traces, {
    ...BASE_LAYOUT,
    xaxis: { ...AXIS, title: "Launch angle (°)" },
    yaxis: { ...AXIS, title: "Launch speed (m/s)", range: [vLo, vHi] },
    shapes,
  }, CONFIG);
}

/** Force plots to relayout when the window resizes. */
export function attachResize(...divIds) {
  const handler = () => divIds.forEach(id => Plotly.Plots.resize(id));
  window.addEventListener("resize", handler);
}
