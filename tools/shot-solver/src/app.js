// Main wiring: read controls → run solver → draw plots → update readout.
// Live recompute is debounced ~120 ms to keep the UI responsive when sliders
// are dragged.

import { solveEnvelope, pickOptimal, tolerances } from "./solver.js";
import { drawTrajectory, drawEnvelope, attachResize } from "./plots.js";
import { wheelRpsForBall, transfer } from "./slip.js";
import { simulateToGoalY } from "./physics.js";
import { generateLUT, toJava, toPreview } from "./lut.js";

const $ = (id) => document.getElementById(id);

const NUM_IDS = [
  "i-distance", "i-hub-z", "i-hub-r", "i-funnel-h", "i-clearance",
  "i-launch-z", "i-launch-x", "i-wheel-r",
  "i-eta", "i-wear", "i-angle-bias",
  "i-g", "i-mass", "i-diameter", "i-rho", "i-cd",
  "i-th-min", "i-th-max", "i-th-n", "i-v-cap",
  "i-lut-min", "i-lut-max", "i-lut-n",
];

function readInputs() {
  const num = (id) => parseFloat($(id).value);
  return {
    geom: {
      distance: num("i-distance"),
      hubZ: num("i-hub-z"),
      hubR: num("i-hub-r"),
      funnelH: num("i-funnel-h"),
      clearance: num("i-clearance"),
      launchZ: num("i-launch-z"),
      launchX: num("i-launch-x"),
    },
    slip: { eta: num("i-eta"), wear: num("i-wear") },
    angleBiasDeg: num("i-angle-bias"),
    wheelRadius: num("i-wheel-r"),
    env: {
      g: num("i-g"),
      mass: num("i-mass"),
      diameter: num("i-diameter"),
      rho: num("i-rho"),
      cd: num("i-cd"),
      dragEnabled: $("i-drag-enable").checked,
    },
    search: {
      thetaMinDeg: num("i-th-min"),
      thetaMaxDeg: num("i-th-max"),
      thetaSteps: Math.max(8, Math.floor(num("i-th-n"))),
      vCap: num("i-v-cap"),
    },
    showAll: $("i-show-all").checked,
  };
}

function fmtAngle(rad) { return (rad * 180 / Math.PI).toFixed(1) + "°"; }
function fmtMps(v) { return v.toFixed(2) + " m/s"; }

function updateSliderLabels() {
  $("o-eta").textContent = parseFloat($("i-eta").value).toFixed(2);
  $("o-wear").textContent = parseFloat($("i-wear").value).toFixed(2);
  const bias = parseFloat($("i-angle-bias").value);
  $("o-angle-bias").textContent = (bias >= 0 ? "+" : "") + bias.toFixed(1);
}

function recompute() {
  updateSliderLabels();
  const inp = readInputs();
  const env = solveEnvelope(inp.geom, inp.search, inp.env);
  const opt = pickOptimal(env);
  const tol = opt ? tolerances(env, opt) : null;

  // Geometry passed to the trajectory drawer needs physics + geometry merged.
  const geomForDraw = { ...inp.geom, ...inp.env };
  drawTrajectory("plot-traj", env, geomForDraw, opt, inp.showAll);
  drawEnvelope("plot-env", env, opt, tol);

  // Readout. We show two angles: the physics-optimal (what the ball actually
  // launches at) and the hood command (physics + slo-mo bias).
  if (opt) {
    const physDeg = opt.thetaRad * 180 / Math.PI;
    const hoodDeg = physDeg + inp.angleBiasDeg;
    $("rd-optimal").textContent =
      inp.angleBiasDeg !== 0
        ? `${physDeg.toFixed(1)}° (hood ${hoodDeg.toFixed(1)}°) @ ${fmtMps(opt.v0)}`
        : `${fmtAngle(opt.thetaRad)} @ ${fmtMps(opt.v0)}`;
    if (tol && tol.speedLow != null && tol.speedHigh != null) {
      $("rd-speed-tol").textContent = `−${tol.speedLow.toFixed(2)} / +${tol.speedHigh.toFixed(2)}`;
    } else { $("rd-speed-tol").textContent = "—"; }
    if (tol && tol.angleLow != null) {
      const lo = (tol.angleLow * 180 / Math.PI).toFixed(1);
      const hi = (tol.angleHigh * 180 / Math.PI).toFixed(1);
      $("rd-angle-tol").textContent = `−${lo}° / +${hi}°`;
    } else { $("rd-angle-tol").textContent = "—"; }
    const revPerSec = wheelRpsForBall(opt.v0, inp.slip, inp.wheelRadius);
    const radPerSec = revPerSec * 2 * Math.PI;
    const wheelMps = opt.v0 / Math.max(transfer(inp.slip), 1e-6);
    $("rd-rps").textContent =
      `${wheelMps.toFixed(2)} m/s · ${revPerSec.toFixed(1)} rev/s · ${radPerSec.toFixed(0)} rad/s`;
    const tof = simulateToGoalY(opt.v0, opt.thetaRad, {
      ...inp.env, x0: 0, y0: inp.geom.launchZ, stopX: env.geom.xFar + 1.0,
    }, inp.geom.hubZ);
    $("rd-tof").textContent = tof.t != null ? `${tof.t.toFixed(3)} s` : "—";
  } else {
    $("rd-optimal").textContent = "no solution";
    $("rd-speed-tol").textContent = "—";
    $("rd-angle-tol").textContent = "—";
    $("rd-rps").textContent = "—";
    $("rd-tof").textContent = "—";
  }
}

// ─── debounce ───────────────────────────────────────────────────────────────
let debounceT = 0;
function scheduleRecompute() {
  clearTimeout(debounceT);
  debounceT = setTimeout(recompute, 120);
}

// ─── LUT generation ─────────────────────────────────────────────────────────
function runLUT() {
  const inp = readInputs();
  const sweep = {
    distMin: parseFloat($("i-lut-min").value),
    distMax: parseFloat($("i-lut-max").value),
    steps: Math.max(2, Math.floor(parseFloat($("i-lut-n").value))),
  };
  const { distance, ...geomBase } = inp.geom;
  const rows = generateLUT(sweep, geomBase, inp.search, inp.env, inp.slip, inp.wheelRadius, inp.angleBiasDeg);
  const java = toJava($("i-lut-name").value || "loadLUTSolved", rows);
  const preview = toPreview(rows);
  $("lut-output").textContent = preview + "\n\n" + java;
  $("lut-output").dataset.java = java;
}

async function copyLUT() {
  const java = $("lut-output").dataset.java || $("lut-output").textContent;
  try {
    await navigator.clipboard.writeText(java);
    flashCopy("copied");
  } catch (e) {
    flashCopy("copy failed");
  }
}
function flashCopy(msg) {
  const b = $("b-copy-lut");
  const old = b.textContent;
  b.textContent = msg;
  setTimeout(() => (b.textContent = old), 1100);
}

// ─── wire events ────────────────────────────────────────────────────────────
function wire() {
  for (const id of NUM_IDS) {
    $(id).addEventListener("input", scheduleRecompute);
    $(id).addEventListener("change", scheduleRecompute);
  }
  $("i-drag-enable").addEventListener("change", scheduleRecompute);
  $("i-show-all").addEventListener("change", scheduleRecompute);
  $("b-gen-lut").addEventListener("click", runLUT);
  $("b-copy-lut").addEventListener("click", copyLUT);
  attachResize("plot-traj", "plot-env");
}

wire();
recompute();
