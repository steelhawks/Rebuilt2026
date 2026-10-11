// /tuner overlay: connect to the robot over NT4, push the LUT the solver
// generated, and confirm the robot applied exactly what we sent.
//
// Importing app.js also runs the solver page wiring (it self-executes wire() +
// recompute() on load), so this is the only module the tuner page loads.
import { buildLUT } from "./app.js";
import { NTClient, T, javaHashCode } from "./nt.js";

const $ = (id) => document.getElementById(id);

// ─── NT keys (leading slash = NT topic path for the robot's LoggedNetwork*) ──
const K = {
  payload:   "/GeneratedLUTTuner/Payload",
  applyReq:  "/GeneratedLUTTuner/ApplyRequest",   // monotonic: write Date.now()
  revertReq: "/GeneratedLUTTuner/RevertRequest",
  target:    "/GeneratedLUTTuner/TargetDistanceMeters",
  hash:      "/GeneratedLUTTuner/AppliedHash",
  status:    "/GeneratedLUTTuner/Status",
  rows:      "/GeneratedLUTTuner/AppliedRows",
  minDist:   "/GeneratedLUTTuner/AppliedMinDistance",
  maxDist:   "/GeneratedLUTTuner/AppliedMaxDistance",
  liveDist:  "/GeneratedLUTTuner/LiveDistanceMeters",
  distErr:   "/GeneratedLUTTuner/DistanceErrorMeters",
  enabled:   "/GeneratedLUTTuner/Enabled",       // write-only (we set the gate)
  enabledEcho: "/GeneratedLUTTuner/EnabledEcho",  // read-only mirror of the gate
  useLut:    "/GeneratedLUTTuner/UseLut",
  forceUseLutReq: "/GeneratedLUTTuner/ForceUseLutRequest",
};

const nt = new NTClient();
let connected = false;
let pendingHash = null;       // hash of the table we last pushed
let robotHash = "";           // hash the robot echoed
let robotStatus = "—";
let ackTimer = 0;             // watchdog: warn if no ack lands

const round = (x, n) => {
  const p = 10 ** n;
  return Math.round((x ?? 0) * p) / p;
};

// ─── connection ──────────────────────────────────────────────────────────────
function connect() {
  const host = $("nt-host").value || "localhost";
  setStatusDot("connecting", `connecting to ${host}…`);
  nt.connect(host, (isConn) => {
    connected = isConn;
    setStatusDot(isConn ? "ok" : "down", isConn ? `connected · ${host}` : `disconnected · ${host}`);
    refreshButtons();
  });
  wireSubscriptions();
}

function setStatusDot(state, text) {
  const dot = $("nt-dot");
  dot.className = "nt-dot " + state;
  $("nt-conn-text").textContent = text;
}

function refreshButtons() {
  const dis = !connected;
  $("nt-push").disabled = dis;
  $("nt-revert").disabled = dis;
  $("nt-set-target").disabled = dis;
  $("nt-enable").disabled = dis;
  $("nt-uselut-on").disabled = dis;
}

// ─── subscriptions (robot → overlay) ─────────────────────────────────────────
function wireSubscriptions() {
  nt.subscribe(K.hash, T.string, (v) => { robotHash = v ?? ""; renderConfirm(); });
  nt.subscribe(K.status, T.string, (v) => { robotStatus = v ?? "—"; renderConfirm(); });
  nt.subscribe(K.rows, T.double, (v) => { $("nt-rows").textContent = v != null ? String(Math.round(v)) : "—"; });
  nt.subscribe(K.minDist, T.double, (v) => { $("nt-min").textContent = fmt(v); });
  nt.subscribe(K.maxDist, T.double, (v) => { $("nt-max").textContent = fmt(v); });
  nt.subscribe(K.liveDist, T.double, (v) => { $("nt-live").textContent = fmt(v) + " m"; renderError(v); });
  nt.subscribe(K.distErr, T.double, (v) => { renderErrorValue(v); });
  nt.subscribe(K.enabledEcho, T.boolean, (v) => {
    $("nt-enable").checked = !!v;
    $("nt-enable-state").textContent = v ? "accepting pushes" : "ignoring pushes";
    $("nt-enable-state").className = "nt-flag " + (v ? "ok" : "warn");
  });
  nt.subscribe(K.useLut, T.boolean, (v) => {
    $("nt-uselut-state").textContent = v ? "useLUT ON" : "useLUT OFF — pushed table won't shoot";
    $("nt-uselut-state").className = "nt-flag " + (v ? "ok" : "warn");
    $("nt-uselut-on").style.display = v ? "none" : "inline-block";
  });
}

const fmt = (v) => (v == null ? "—" : Number(v).toFixed(3));

let lastLive = null;
function renderError(live) { lastLive = live; }
function renderErrorValue(err) {
  const target = parseFloat($("nt-target").value);
  if (!target) { $("nt-error").textContent = "set a target"; $("nt-error").className = "nt-flag"; return; }
  if (err == null) { $("nt-error").textContent = "—"; return; }
  const mag = Math.abs(err);
  const dir = err > 0 ? "too far — drive in" : "too close — back up";
  $("nt-error").textContent = `${err >= 0 ? "+" : ""}${err.toFixed(2)} m · ${mag < 0.1 ? "✓ on target" : dir}`;
  $("nt-error").className = "nt-flag " + (mag < 0.1 ? "ok" : "warn");
}

// ─── confirmation ────────────────────────────────────────────────────────────
function renderConfirm() {
  const box = $("nt-confirm");
  $("nt-status").textContent = robotStatus;
  if (pendingHash == null) {
    box.className = "nt-confirm";
    box.textContent = "No table pushed yet.";
    return;
  }
  if (robotHash && robotHash === pendingHash && robotStatus === "OK") {
    clearTimeout(ackTimer);
    box.className = "nt-confirm ok";
    box.textContent = "✓ Robot is running this table (hash matched).";
  } else if (robotStatus.startsWith("error")) {
    box.className = "nt-confirm err";
    box.textContent = "✗ Robot rejected the push: " + robotStatus;
  } else {
    box.className = "nt-confirm wait";
    box.textContent = "Pushing… waiting for robot to ack (hash match).";
  }
}

// ─── push / revert / target ──────────────────────────────────────────────────
async function push() {
  const { rows } = buildLUT();
  const ok = rows.filter((r) => r.ok);
  if (ok.length < 2) {
    pendingHash = null;
    $("nt-confirm").className = "nt-confirm err";
    $("nt-confirm").textContent = "Solver produced <2 valid rows — widen the distance range or fix inputs.";
    return;
  }
  const payload = {
    rows: ok.map((r) => ({
      d: round(r.distance, 4),
      flywheel: round(r.wheelMps, 3),
      hood: round(r.hoodDeg, 2),
      tof: round(r.tof ?? 0, 4),
      close: round(r.wheelCloseMps, 3),
      far: round(r.wheelFarMps, 3),
    })),
  };
  const json = JSON.stringify(payload);
  pendingHash = String(javaHashCode(json));
  robotHash = "";
  robotStatus = "—";
  renderConfirm();
  try {
    // Payload must land before the apply pulse so the robot reads it on the
    // same loop it consumes the trigger.
    await nt.set(K.payload, T.string, json);
    await nt.set(K.applyReq, T.double, Date.now());
  } catch (e) {
    $("nt-confirm").className = "nt-confirm err";
    $("nt-confirm").textContent = "Push failed: " + e.message;
    return;
  }
  clearTimeout(ackTimer);
  ackTimer = setTimeout(() => {
    if (robotHash !== pendingHash) {
      $("nt-confirm").className = "nt-confirm err";
      $("nt-confirm").textContent =
        "No ack from robot. Check: gate 'Accept pushes' is ON, the robot code is deployed/running, and you're on the right host.";
    }
  }, 4000);
}

async function revert() {
  pendingHash = null;
  $("nt-confirm").className = "nt-confirm wait";
  $("nt-confirm").textContent = "Reverting to baked LUT…";
  try {
    await nt.set(K.revertReq, T.double, Date.now());
  } catch (e) {
    $("nt-confirm").className = "nt-confirm err";
    $("nt-confirm").textContent = "Revert failed: " + e.message;
  }
}

async function setTarget() {
  const v = parseFloat($("nt-target").value);
  try { await nt.set(K.target, T.double, isFinite(v) ? v : 0); }
  catch (e) { console.error("set target failed", e); }
}

// ─── wire ────────────────────────────────────────────────────────────────────
$("nt-connect").addEventListener("click", connect);
$("nt-push").addEventListener("click", push);
$("nt-revert").addEventListener("click", revert);
$("nt-set-target").addEventListener("click", setTarget);
$("nt-enable").addEventListener("change", async (e) => {
  try { await nt.set(K.enabled, T.boolean, e.target.checked); }
  catch (err) { console.error("enable toggle failed", err); }
});
$("nt-uselut-on").addEventListener("click", async () => {
  try { await nt.set(K.forceUseLutReq, T.double, Date.now()); }
  catch (err) { console.error("force useLUT failed", err); }
});

refreshButtons();
// Auto-connect to the default (sim) on load for convenience.
connect();
