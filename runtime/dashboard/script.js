// =====================================================================
// car-calib dashboard frontend
// Sections (search by banner):
//   §1  Config + DOM refs
//   §2  Script step editor (form, render, edit/remove)
//   §3  Run/Stop + relay light
//   §4  Status renderers (header pills, status bar, metric grid)
//   §5  Event log
//   §6  Tabs (zone-2 + modal)
//   §7  Progress + step elapsed
//   §8  Polling (status / route_script status)
//   §9  Routes table (filter / sort / search / bulk)
//  §10  Route summary modal
//  §11  Presets CRUD
//  §12  Tune panel (steering controller params)
//  §13  Map tab (tile-map editor like ARL_AUTOCAR map survey + live cell)
//  §14  Pre-calibrate tab (pre-run checklist)
// =====================================================================
(function () {
"use strict";

// §1 ─────────────────────────────────────────────────────────────────
const TOKEN = window.DASHBOARD_CONFIG.TOKEN;
const STREAM = window.DASHBOARD_CONFIG.STREAM;
const STATUS = window.DASHBOARD_CONFIG.STATUS;
const qp = TOKEN ? ("?token=" + encodeURIComponent(TOKEN)) : "";
document.getElementById("stream").src = STREAM + qp;

const steps = [];
const tbody = document.querySelector("#steps tbody");
const preview = document.getElementById("preview");
const runPill = document.getElementById("runPill");
const runDetail = document.getElementById("runDetail");
const progressBar = document.getElementById("progressBar");
const progressSegments = document.getElementById("progressSegments");
const actionInput = document.getElementById("action");
const durationInput = document.getElementById("duration");
const actionDistanceInput = document.getElementById("actionDistance");
const avgSpeedLabel = document.getElementById("avgSpeedLabel");
const addBtn = document.getElementById("add");
const cancelEditBtn = document.getElementById("cancelEdit");
const manualJoystick = document.getElementById("manualJoystick");
const manualJoystickKnob = document.getElementById("manualJoystickKnob");
const manualPill = document.getElementById("manualPill");
const manualDetail = document.getElementById("manualDetail");
let currentRunningStep = 0;
let isRunning = false;
let isRecording = false;
let editingIndex = -1;
let manualActive = false;
let manualPointerId = null;
let manualDrive = 0;
let manualSteer = 0;
let manualHeartbeat = null;

// Lets the operator type a target distance (meters) instead of guessing a
// duration by hand -- duration = distance / avg speed measured from past
// completed routes' manually-entered distance_m (see compute_average_speed_mps).
let _avgSpeedMps = null;
let _speedInterceptM = 0;
async function refreshAvgSpeed() {
  try {
    const r = await fetch("/routes/avg_speed" + qp);
    if (!r.ok) return;
    const j = await r.json();
    _avgSpeedMps = j.avg_speed_mps;
    _speedInterceptM = j.intercept_m || 0;
    if (avgSpeedLabel) {
      avgSpeedLabel.textContent = _avgSpeedMps
        ? `TB: ${_avgSpeedMps.toFixed(3)} m/s (${j.n_samples} route)`
        : "chưa đủ dữ liệu tốc độ";
    }
  } catch (e) {}
}
if (actionDistanceInput) actionDistanceInput.addEventListener("input", () => {
  const distance = parseFloat(actionDistanceInput.value);
  if (!Number.isFinite(distance) || distance <= 0 || !_avgSpeedMps) return;
  // duration = (distance - startup_offset) / speed -- accounts for the
  // roughly-constant distance lost accelerating before reaching cruise
  // speed, which a flat distance/speed division was undershooting by a
  // few cm on short steps (see compute_average_speed_mps docstring).
  const duration = (distance - _speedInterceptM) / _avgSpeedMps;
  durationInput.value = Math.max(0, duration).toFixed(1);
});
refreshAvgSpeed();

// Script modes: 1 time (every step by seconds), 2 tiles (forward/straight
// steps by tile count, turns by seconds; typed or built from the saved map
// route), 3 A* (the Pi builds the steps from the saved start -> goal).
const scriptModeSel = document.getElementById("scriptMode");
const durationUnit = document.getElementById("durationUnit");
const TILE_ACTIONS = new Set(["forward", "straight"]);
// Turns can run by degrees: the Pi stops them when the grid heading has seen the angle.
const DEGREE_ACTIONS = new Set(["left", "right", "turn_left", "turn_right"]);
const turnUnitSel = document.getElementById("turnUnit");
function stepUsesTiles(action) {
  return scriptModeSel && scriptModeSel.value !== "time" && TILE_ACTIONS.has(action);
}
function stepUsesDegrees(action) {
  return !!turnUnitSel && DEGREE_ACTIONS.has(action) && turnUnitSel.value.startsWith("deg");
}
function stepAmount(s) {
  if (s.tiles != null) return `${s.tiles} ô`;
  if (s.degrees != null) return `${Number(s.degrees).toFixed(0)}°` + (s.anchor ? " @V+1" : "");
  return `${Number(s.duration_s).toFixed(1)} s`;
}
function updateDurationUnit() {
  if (turnUnitSel) turnUnitSel.style.display = DEGREE_ACTIONS.has(actionInput.value) ? "" : "none";
  if (durationUnit) {
    durationUnit.textContent = stepUsesTiles(actionInput.value) ? "ô (tiles)"
      : (stepUsesDegrees(actionInput.value) ? ""
      : (actionInput.value.startsWith("lane_") ? "s (timeout: lâu nhất để qua vạch)" : "seconds"));
    durationUnit.style.display = DEGREE_ACTIONS.has(actionInput.value) ? "none" : "";
  }
}
// Turning presets for A*: each turn the planner emits (plus the settle that
// follows it) is replaced by the steps of the route preset picked for that side.
const TURN_PRESET_SIDES = {left: ["turn_left", "left"], right: ["turn_right", "right"]};
const turnPresetSel = {left: document.getElementById("turnPresetLeft"), right: document.getElementById("turnPresetRight")};
let turnPresetChoice = {left: "", right: ""};
try { turnPresetChoice = Object.assign(turnPresetChoice, JSON.parse(localStorage.getItem("turnPresets") || "{}")); } catch (e) { /* private window */ }

function presetStep(s) {
  if (s.tiles != null) return {action: s.action, tiles: Number(s.tiles)};
  if (s.degrees != null) return Object.assign({action: s.action, degrees: Number(s.degrees)}, s.anchor ? {anchor: s.anchor} : {});
  return {action: s.action, duration_s: Number(s.duration_s)};
}

function expandTurns(planSteps, bySide) {
  const out = [];
  let replaced = 0;
  for (let i = 0; i < planSteps.length; i++) {
    const s = planSteps[i];
    const side = Object.keys(TURN_PRESET_SIDES).find(k => TURN_PRESET_SIDES[k].includes(s.action));
    const preset = side ? bySide[side] : null;
    if (!preset) { out.push(s); continue; }
    preset.forEach(p => out.push(presetStep(p)));
    replaced++;
    if (planSteps[i + 1] && planSteps[i + 1].action === "settle") i++;  // the preset brings its own settle
  }
  return {steps: out, replaced};
}

async function fetchPresetSteps(name) {
  if (!name) return null;
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`);
  if (!r.ok) throw new Error(`preset "${name}" not found`);
  const j = await r.json();
  return ((j.preset || {}).steps || []);
}

async function loadStepsFromMap() {
  const hint = document.getElementById("scriptModeHint");
  try {
    const r = await fetch("/api/map/saved_steps" + qp);
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.detail || r.status);
    const bySide = {
      left: await fetchPresetSteps(turnPresetChoice.left),
      right: await fetchPresetSteps(turnPresetChoice.right),
    };
    const turns = j.steps.filter(s => s.action.startsWith("turn") || s.degrees != null).length;
    const expanded = expandTurns(j.steps, bySide);
    steps.length = 0;
    expanded.steps.forEach(s => steps.push(s));
    resetEditor();
    render();
    const tiles = j.steps.reduce((n, s) => n + (s.tiles || 0), 0);
    const used = ["left", "right"].filter(k => turnPresetChoice[k]).map(k => `${k === "left" ? "trái" : "phải"} = ${turnPresetChoice[k]}`);
    hint.textContent = `A* từ map ${j.map_id}: ${j.path.length - 1} ô, ${turns} lần rẽ (${tiles} ô forward)` +
      (used.length ? `, ${expanded.replaced} lần rẽ dùng preset (${used.join(", ")})` : "") + ". Sửa được từng action trước khi chạy.";
  } catch (e) {
    hint.textContent = "lỗi: " + e.message;
  }
}
function applyScriptMode() {
  const mode = scriptModeSel.value;
  const hint = document.getElementById("scriptModeHint");
  const fromMap = document.getElementById("stepsFromMap");
  fromMap.style.display = mode === "time" ? "none" : "";
  fromMap.textContent = mode === "astar" ? "tính lại A*" : "tạo từ map";
  const turnPresetBox = document.getElementById("turnPresetBox");
  if (turnPresetBox) turnPresetBox.style.display = mode === "astar" ? "" : "none";
  hint.textContent = mode === "time" ? "" : (mode === "tiles"
    ? "forward/straight đi theo số ô (bộ đếm vạch ngang), rẽ vẫn theo giây"
    : "");
  updateDurationUnit();
  if (mode === "astar") loadStepsFromMap();
}
if (scriptModeSel) {
  scriptModeSel.onchange = applyScriptMode;
  actionInput.addEventListener("change", updateDurationUnit);
  if (turnUnitSel) turnUnitSel.addEventListener("change", updateDurationUnit);
  document.getElementById("stepsFromMap").onclick = loadStepsFromMap;
  Object.entries(turnPresetSel).forEach(([side, sel]) => {
    if (!sel) return;
    sel.onchange = () => {
      turnPresetChoice[side] = sel.value;
      try { localStorage.setItem("turnPresets", JSON.stringify(turnPresetChoice)); } catch (e) { /* private window */ }
      if (scriptModeSel.value === "astar") loadStepsFromMap();
    };
  });
}

function resetEditor() {
  editingIndex = -1;
  addBtn.textContent = "+ add step";
  cancelEditBtn.style.display = "none";
}

function loadEditorFromStep(idx) {
  const step = steps[idx];
  if (!step) return;
  editingIndex = idx;
  actionInput.value = step.action;
  durationInput.value = String(step.tiles != null ? step.tiles : (step.degrees != null ? step.degrees : step.duration_s));
  if (turnUnitSel) turnUnitSel.value = step.degrees != null ? (step.anchor ? "deg_anchor" : "deg") : "s";
  updateDurationUnit();
  addBtn.textContent = "save edit";
  cancelEditBtn.style.display = "";
}

function render() {
  tbody.innerHTML = "";
  steps.forEach((s, i) => {
    const tr = document.createElement("tr");
    tr.className = "step-row";
    tr.dataset.idx = i;
    if (isRunning) {
      if (i + 1 < currentRunningStep) tr.classList.add("done");
      else if (i + 1 === currentRunningStep) tr.classList.add("active");
      else tr.classList.add("pending");
    }
    tr.innerHTML = `<td class="idx">${i+1}</td><td>${safeText(s.action)}</td><td>${stepAmount(s)}</td><td>${isRunning ? "" : `<button data-i="${i}" class="play" title="run only this step">▶</button> <button data-i="${i}" class="edit">edit</button> <button data-i="${i}" class="rm">×</button>`}</td>`;
    tbody.appendChild(tr);
  });
  preview.textContent = JSON.stringify({steps}, null, 2);
}

addBtn.onclick = () => {
  if (isRunning) return;
  const action = actionInput.value;
  const amount = parseFloat(durationInput.value || "0");
  let step;
  if (stepUsesTiles(action)) {
    if (!Number.isInteger(amount) || amount < 1) { runDetail.textContent = "số ô phải là số nguyên ≥ 1"; return; }
    step = {action, tiles: amount};
  } else if (stepUsesDegrees(action)) {
    if (!isFinite(amount) || amount < 5 || amount > 180) { runDetail.textContent = "góc phải từ 5 đến 180 độ"; return; }
    step = {action, degrees: amount};
    if (turnUnitSel.value === "deg_anchor") step.anchor = "next_line";
  } else {
    if (!isFinite(amount) || amount <= 0) { runDetail.textContent = "duration phải > 0"; return; }
    step = {action, duration_s: amount};
  }
  if (editingIndex >= 0 && editingIndex < steps.length) {
    steps[editingIndex] = step;
    resetEditor();
  } else {
    steps.push(step);
  }
  render();
};

cancelEditBtn.onclick = () => {
  if (isRunning) return;
  resetEditor();
};
document.getElementById("clear").onclick = () => {
  if (isRunning) return;
  steps.length = 0;
  resetEditor();
  render();
};
tbody.onclick = (e) => {
  if (isRunning) return;
  const t = e.target;
  const idx = parseInt(t.dataset.i, 10);
  if (!Number.isFinite(idx)) return;

  if (t.classList.contains("play")) {
    playSingleStep(idx);
    return;
  }

  if (t.classList.contains("edit")) {
    loadEditorFromStep(idx);
    return;
  }

  if (t.classList.contains("rm")) {
    steps.splice(idx, 1);
    if (editingIndex === idx) resetEditor();
    else if (editingIndex > idx) editingIndex -= 1;
    render();
  }
};

async function playSingleStep(idx) {
  const step = steps[idx];
  if (!step) return;
  if (editingIndex >= 0) { runDetail.textContent = "save edit first"; return; }
  runDetail.textContent = `step ${idx + 1}: starting…`;
  try {
    const r = await fetch("/route/script/step" + qp, {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify(step),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) {
      runDetail.textContent = "step error: " + (j.detail || r.status);
      return;
    }
    runDetail.textContent = "";
  } catch (e) {
    runDetail.textContent = "step network error";
  }
}

const runBtn = document.getElementById("run");
const recordBtn = document.getElementById("record");

// Both buttons toggle: start while idle, stop while active. State comes from
// /route/script/status polling (see pollStatus), plus an optimistic flip on
// a successful click so the label changes without waiting for the next poll.
function renderRunButtons() {
  runBtn.textContent = isRunning ? "■ dừng route" : "▶ bắt đầu route";
  runBtn.className = isRunning ? "danger" : "primary";
  recordBtn.textContent = isRecording ? "■ dừng record" : "● bắt đầu record";
  recordBtn.className = isRecording ? "danger" : "primary";
}

function applyRecordState(rec) {
  const wasRecording = isRecording;
  isRecording = !!(rec && rec.active);
  renderRunButtons();
  if (wasRecording && !isRecording) {
    setTimeout(refreshRoutes, 800);
  }
}

runBtn.onclick = async () => {
  if (isRunning) {
    try {
      await fetch("/route/script/stop" + qp, {method: "POST"});
      runDetail.textContent = "stopping…";
    } catch (e) {
      runDetail.textContent = "stop network error";
    }
    return;
  }
  if (editingIndex >= 0) { runDetail.textContent = "save edit first"; return; }
  if (steps.length === 0) { runDetail.textContent = "add steps first"; return; }
  if (!(await precalGateOk())) { runDetail.textContent = "huỷ: pre-calibrate chưa đạt"; return; }
  runDetail.textContent = "submitting…";
  const presetSel = document.getElementById("presetSelect");
  const presetInput = document.getElementById("presetName");
  const preset_name = (presetSel && presetSel.value) || (presetInput && presetInput.value.trim()) || null;
  const body = {steps, preset_name, description: null, mode: scriptModeSel ? scriptModeSel.value : "time"};
  const r = await fetch("/route/script" + qp, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { runDetail.textContent = "error: " + JSON.stringify(j.detail || r.status); return; }
  runDetail.textContent = "";
  isRunning = true;
  renderRunButtons();
  render();
};

recordBtn.onclick = async () => {
  const path = isRecording ? "/route/record/stop" : "/route/record/start";
  try {
    const r = await fetch(path + qp, {method: "POST"});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { runDetail.textContent = "record error: " + (j.detail || r.status); return; }
    applyRecordState(j.record);
  } catch (e) {
    runDetail.textContent = "record network error";
  }
};

async function setLight(on) {
  const sep = qp ? "&" : "?";
  const toggle = document.getElementById("lightToggle");
  const label = document.getElementById("lightLabel");
  try {
    const r = await fetch("/route/relay" + qp + sep + "on=" + (on ? 1 : 0), {method: "POST"});
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      label.textContent = "light error: " + (j.detail || j.error || r.status);
      toggle.checked = !on;  // revert switch on failure
      return;
    }
    label.textContent = on ? "light on" : "light off";
  } catch (e) {
    label.textContent = "light error: network";
    toggle.checked = !on;
  }
}
document.getElementById("lightToggle").onchange = (e) => setLight(e.target.checked);

let _powerBusy = false;
async function sendPower(on) {
  const sep = qp ? "&" : "?";
  const label = document.getElementById("powerLabel");
  const onBtn = document.getElementById("powerOnBtn");
  const offBtn = document.getElementById("powerOffBtn");
  if (_powerBusy) return;  // debounce: a pulse request is already in flight
  if (!on && !confirm("Tắt xe? Relay sẽ chập giữ 3s.")) { label.textContent = ""; return; }
  _powerBusy = true;
  onBtn.disabled = true; offBtn.disabled = true;
  label.textContent = on ? "đang bật…" : "đang tắt…";
  try {
    const r = await fetch("/control/power" + qp + sep + "on=" + (on ? 1 : 0), {method: "POST"});
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      label.textContent = "power error: " + (j.detail || j.error || r.status);
      return;
    }
    label.textContent = on ? "đã gửi BẬT" : "đã gửi TẮT";
    setTimeout(() => { label.textContent = ""; }, 2000);
  } catch (e) {
    label.textContent = "power error: network";
  } finally {
    // Hold the lock long enough to cover the longest (OFF) pulse so a rapid
    // second click cannot overlap the relay pulse on the actuator side.
    setTimeout(() => {
      _powerBusy = false;
      updateActuatorControls({stale: false, online: true, estop: false});
    }, 3500);
  }
}
document.getElementById("powerOnBtn").onclick = () => sendPower(true);
document.getElementById("powerOffBtn").onclick = () => sendPower(false);

// §3b ── Camera select combo box ────────────────────────────────────────
const cameraSelect = document.getElementById("cameraSelect");
const cameraLabel = document.getElementById("cameraLabel");
let cameraBusy = false;

async function refreshCameraList() {
  try {
    const r = await fetch("/api/camera/list" + qp);
    if (!r.ok) {
      cameraLabel.textContent = "camera scan error: " + r.status;
      return;
    }
    const data = await r.json();
    const cameras = data.cameras || [];
    cameraSelect.innerHTML = "";
    if (cameras.length === 0) {
      cameraSelect.appendChild(new Option("no camera found", ""));
      cameraLabel.textContent = "";
      return;
    }
    for (const idx of cameras) {
      cameraSelect.appendChild(new Option("camera " + idx + " (/dev/video" + idx + ")", idx));
    }
    cameraSelect.value = String(data.current);
    cameraLabel.textContent = "active: /dev/video" + data.current;
  } catch (e) {
    cameraLabel.textContent = "camera scan error: network";
  }
}

async function selectCamera(index) {
  if (cameraBusy) return;
  cameraBusy = true;
  cameraLabel.textContent = "switching…";
  try {
    const r = await fetch("/api/camera/select" + qp, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({index: Number(index)}),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) {
      cameraLabel.textContent = "camera error: " + (j.detail || r.status);
      return;
    }
    cameraLabel.textContent = "switching to /dev/video" + index + "…";
  } catch (e) {
    cameraLabel.textContent = "camera error: network";
  } finally {
    cameraBusy = false;
  }
}

cameraSelect.onchange = (e) => selectCamera(e.target.value);
document.getElementById("cameraRefresh").onclick = refreshCameraList;
refreshCameraList();

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function setManualUi(cls, text, detail) {
  if (manualPill) {
    manualPill.className = "pill " + cls;
    manualPill.textContent = text;
  }
  if (manualDetail) manualDetail.textContent = detail || "";
}

function updateJoystickKnob(drive, steer) {
  if (!manualJoystickKnob || !manualJoystick) return;
  const rect = manualJoystick.getBoundingClientRect();
  const radius = Math.max(1, rect.width / 2 - 24);
  const x = steer * radius;
  const y = -drive * radius;
  manualJoystickKnob.style.transform = `translate(calc(-50% + ${x.toFixed(1)}px), calc(-50% + ${y.toFixed(1)}px))`;
}

function joystickAxesFromEvent(e) {
  const rect = manualJoystick.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const radius = Math.max(1, rect.width / 2 - 24);
  return {
    drive: clamp(-(e.clientY - cy) / radius, -1, 1),
    steer: clamp((e.clientX - cx) / radius, -1, 1),
  };
}

async function sendManualOverride(active) {
  const body = {
    active: !!active,
    drive: active ? manualDrive : 0,
    steer: active ? manualSteer : 0,
    ts: Date.now() / 1000,
  };
  try {
    const r = await fetch("/api/manual_override" + qp, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body),
    });
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      setManualUi("pill-error", "manual error", j.detail || String(r.status));
      return;
    }
    if (active) {
      setManualUi("pill-running", "manual active", `drive ${manualDrive.toFixed(2)} · steer ${manualSteer.toFixed(2)}`);
    }
  } catch (e) {
    setManualUi("pill-error", "manual network", "release joystick");
  }
}

function startManual(e) {
  if (!manualJoystick || manualActive) return;
  manualActive = true;
  manualPointerId = e.pointerId;
  manualJoystick.setPointerCapture(e.pointerId);
  manualJoystick.classList.add("active");
  const axes = joystickAxesFromEvent(e);
  manualDrive = axes.drive;
  manualSteer = axes.steer;
  updateJoystickKnob(manualDrive, manualSteer);
  sendManualOverride(true);
  manualHeartbeat = setInterval(() => sendManualOverride(true), 100);
}

function moveManual(e) {
  if (!manualActive || e.pointerId !== manualPointerId) return;
  const axes = joystickAxesFromEvent(e);
  manualDrive = axes.drive;
  manualSteer = axes.steer;
  updateJoystickKnob(manualDrive, manualSteer);
}

function stopManual() {
  if (!manualActive) return;
  const pointerId = manualPointerId;
  manualActive = false;
  manualPointerId = null;
  manualDrive = 0;
  manualSteer = 0;
  if (manualHeartbeat) clearInterval(manualHeartbeat);
  manualHeartbeat = null;
  if (manualJoystick) {
    if (pointerId !== null && manualJoystick.hasPointerCapture && manualJoystick.hasPointerCapture(pointerId)) {
      manualJoystick.releasePointerCapture(pointerId);
    }
    manualJoystick.classList.remove("active");
  }
  updateJoystickKnob(0, 0);
  setManualUi("pill-idle", "idle", "released");
  sendManualOverride(false);
}

function renderManualStatus(tel) {
  if (manualActive) return;
  const t = tel || {};
  const reason = t.manual_blocked_reason || "";
  if (reason === "timeout") {
    setManualUi("pill-error", "manual timeout", "base stopped");
  } else if (reason) {
    setManualUi("pill-paused", "blocked", reason);
  } else if (t.manual_override_active) {
    setManualUi("pill-running", "manual active", `drive ${Number(t.manual_drive || 0).toFixed(2)} · steer ${Number(t.manual_steer || 0).toFixed(2)}`);
  } else {
    setManualUi("pill-idle", "idle", "");
  }
}

if (manualJoystick) {
  manualJoystick.addEventListener("pointerdown", startManual);
  manualJoystick.addEventListener("pointermove", moveManual);
  manualJoystick.addEventListener("pointerup", stopManual);
  manualJoystick.addEventListener("pointercancel", stopManual);
  window.addEventListener("blur", stopManual);
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopManual(); });
  updateJoystickKnob(0, 0);
}

document.querySelectorAll(".tab").forEach(btn => {
  btn.onclick = () => {
    const target = btn.dataset.tab;
    document.querySelectorAll(".tab").forEach(t => t.classList.toggle("active", t.dataset.tab === target));
    document.querySelectorAll(".tab-pane").forEach(p => p.classList.toggle("active", p.dataset.pane === target));
  };
});

function renderProgressSegments(total, currentIdx, fillRatio) {
  if (!progressSegments) return;
  if (!total) { progressSegments.innerHTML = ""; progressSegments.style.gridTemplateColumns = ""; return; }
  progressSegments.style.gridTemplateColumns = `repeat(${total}, 1fr)`;
  let html = "";
  for (let i = 1; i <= total; i++) {
    if (i < currentIdx) html += '<div class="seg done"></div>';
    else if (i === currentIdx) html += `<div class="seg active" style="--seg-fill:${Math.round((fillRatio||0)*100)}%"></div>`;
    else html += '<div class="seg"></div>';
  }
  progressSegments.innerHTML = html;
}

function setPill(klass, text) {
  runPill.className = "pill " + klass;
  runPill.textContent = text;
}

function safeText(v) {
  if (v === null || v === undefined || v === "") return "-";
  return String(v).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
}

function fmtAge(s) {
  if (s == null || !Number.isFinite(Number(s))) return "-";
  const v = Number(s);
  if (v < 1) return Math.round(v * 1000) + " ms";
  if (v < 60) return v.toFixed(1) + " s";
  return Math.floor(v / 60) + "m " + Math.floor(v % 60) + "s";
}

function fmtUptime(seconds) {
  if (seconds == null || !Number.isFinite(Number(seconds))) return "-";
  const total = Math.max(0, Math.floor(Number(seconds)));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return `${String(h).padStart(2,"0")}:${String(m).padStart(2,"0")}:${String(s).padStart(2,"0")}`;
}

function detectMode(payload) {
  const src = payload && payload.source;
  if (src === "esp32" || src === "esp8266") {
    return {kind: "esp", label: `${src.toUpperCase()} · MQTT`};
  }
  return {kind: "mqtt", label: "MQTT actuator"};
}

function renderStatusBar(rpi, actuator) {
  const payload = (rpi && rpi.payload) || {};
  const mode = detectMode(payload);
  const isEsp = mode.kind === "esp";
  const stale = !rpi || !!rpi.stale;
  const online = !!(rpi && rpi.online);
  const estop = !!payload.estop_active;
  const estopCls = estop ? "text-bad" : "text-ok";
  const estopTxt = estop ? "ACTIVE" : "CLEAR";
  const rpiCls = online ? "text-ok" : "text-bad";
  const rpiTxt = online ? "ONLINE" : "OFFLINE";
  const rpiSub = fmtAge(rpi && rpi.age_s);
  const mqttConnected = payload.mqtt_connected !== false && !stale;
  const mqttTxt = stale ? "stale" : (mqttConnected ? "connected" : "disconnected");
  const mqttCls = mqttConnected ? "text-ok" : "text-bad";
  const pigpio = payload.pigpio_connected;
  const pigpioTxt = stale ? "unknown" : (pigpio ? "ok" : "error");
  const pigpioCls = !stale && pigpio ? "text-ok" : "text-bad";

  // sticky header pills (always visible)
  setHeaderPill("modePill", isEsp ? "warn" : "ok", "Mode", mode.label);
  setHeaderPill("hPillEstop", estop ? "bad" : "ok", "E-stop", estopTxt);
  setHeaderPill("hPillRpi", online ? "ok" : "bad", "Actuator", rpiTxt, rpiSub);
  setHeaderPill("hPillMqtt", mqttConnected ? "ok" : "bad", "MQTT", mqttTxt);

  // Power controls publish to backend; OFF must remain available even if telemetry flaps.
  updateActuatorControls({stale, online, estop});

  const ctrlLabel = isEsp ? "ESP controller" : "Actuator";
  const fourthCell = `<div class="status-cell"><div class="label">GPIO</div><div class="value ${pigpioCls}">${pigpioTxt}</div></div>`;

  document.getElementById("statusBar").innerHTML = `
    <div class="status-grid">
      <div class="status-cell"><div class="label">E-stop</div><div class="value ${estopCls}">${estopTxt}</div></div>
      <div class="status-cell"><div class="label">${ctrlLabel}</div><div class="value ${rpiCls}">${rpiTxt}</div><div class="sub">${safeText(rpiSub)}</div></div>
      <div class="status-cell"><div class="label">MQTT</div><div class="value ${mqttCls}">${mqttTxt}</div></div>
      ${fourthCell}
    </div>
    <div style="margin-top:4px;font-size:12px;color:#666;text-align:center">
      ${fmtUptime(payload.uptime_s ?? payload.uptime_sec)} · ${safeText(payload.hostname || (isEsp ? "ESP serial bridge" : ""))}
    </div>`;
}

function setHeaderPill(id, cls, label, value, sub) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = "h-pill " + (cls || "");
  const lbl = el.querySelector(".lbl");
  if (lbl) lbl.textContent = `${label} ${value}`;
  const subEl = el.querySelector(".sub");
  if (subEl) subEl.textContent = sub || "";
}

function updateActuatorControls({stale, online, estop}) {
  const reachable = !!online && !stale;
  const onBtn = document.getElementById("powerOnBtn");
  const offBtn = document.getElementById("powerOffBtn");
  const lightToggle = document.getElementById("lightToggle");
  // Power controls publish a request to the backend. Do not lock them behind
  // telemetry freshness, because ESP/RPi status can briefly flap while the
  // operator still needs to send ON/OFF. E-stop still blocks ON; OFF stays
  // available so the car can always be shut down.
  if (onBtn) onBtn.disabled = _powerBusy || estop;
  if (offBtn) offBtn.disabled = _powerBusy;
  if (lightToggle) lightToggle.disabled = !reachable;
}

function renderMetricGrid(tel, rpiPayload, actuator) {
  const t = tel || {};
  const p = rpiPayload || {};
  const fsm = safeText(t.fsm_state);
  const theta = t.theta != null ? Number(t.theta).toFixed(2) + "°" : "-";
  const visionServo = t.servo_angle != null ? Number(t.servo_angle).toFixed(1) + "°" : "-";
  const frame = safeText(t.frame_num);
  const base = safeText(p.base_state || p.base);
  const rpiSteer = (p.steer_angle != null) ? Number(p.steer_angle).toFixed(1) + "°" : "-";
  const relay = p.relay_on ? "ON" : "OFF";
  const relayCls = p.relay_on ? "text-warn" : "";
  const mode = safeText(p.current_route_mode || t.route_mode);
  const modeCls = mode === "AUTO" ? "text-ok" : (mode === "-" ? "" : "text-warn");
  const route = safeText(t.route_id || "-");
  const objectState = safeText(t.object_state || p.object_state || "none");
  const objectError = t.object_detector_error || p.object_detector_error || "";
  const objectCls = objectError ? "text-bad" : (objectState === "near" ? "text-bad" : (objectState === "far" ? "text-warn" : "text-ok"));
  // actuator label tracks the active path (ESP USB vs RPi GPIO)
  const actMode = detectMode(p);
  const steerLabel = actMode.kind === "esp" ? "Steer (ESP)" : "Steer (actuator)";

  const cells = [
    {label: "FSM", value: fsm},
    {label: "Theta", value: theta},
    {label: "Servo (vision)", value: visionServo},
    {label: steerLabel, value: rpiSteer},
    {label: "Base", value: base},
    {label: "Object", value: objectError ? "error" : objectState, cls: objectCls},
    {label: "Relay", value: relay, cls: relayCls},
    {label: "Mode", value: mode, cls: modeCls},
    {label: "Frame", value: frame},
    laneShiftCell(t),
    {label: "Route", value: route, full: true},
  ];

  document.getElementById("metricGrid").innerHTML = `
    <div class="metric-section-label">Live Metrics</div>
    <div class="metric-grid">
      ${cells.map(c => `
        <div class="metric-cell ${c.full ? 'metric-cell-full' : ''}">
          <div class="metric-label">${c.label}</div>
          <div class="metric-value ${c.cls || ''}">${c.value}</div>
        </div>`).join("")}
    </div>`;
}

// Last lane shift from vision/lane_shift.py: side + active/passive, and how
// many passive ones (drifts) this run. A new shift also goes to the event log.
let _lastLaneShiftT = null;
function laneShiftCell(t) {
  const n = t.lane_shift_passive_count != null ? t.lane_shift_passive_count : 0;
  if (t.lane_shift_t != null && t.lane_shift_t !== _lastLaneShiftT) {
    if (_lastLaneShiftT !== null) addEvent(t.lane_shift_kind === "passive" ? "warn" : "info", `lane shift ${t.lane_shift_side} (${t.lane_shift_kind})`);
    _lastLaneShiftT = t.lane_shift_t;
  }
  if (!t.lane_shift_side) return {label: "Lane shift", value: `- · drift ${n}`};
  const cls = t.lane_shift_kind === "passive" ? "text-warn" : "text-ok";
  return {label: "Lane shift", value: `${safeText(t.lane_shift_side)} ${safeText(t.lane_shift_kind)} · drift ${n}`, cls};
}

const _eventLog = [];
const MAX_EVENTS = 8;
function addEvent(kind, msg) {
  const ts = new Date().toLocaleTimeString();
  _eventLog.unshift({kind, msg, ts});
  if (_eventLog.length > MAX_EVENTS) _eventLog.pop();
  renderEventLog();
}
function renderEventLog() {
  document.getElementById("eventLog").innerHTML = `
    <div style="font-size:12px;color:#888;text-transform:uppercase;letter-spacing:0.5px;margin-bottom:2px">Events</div>
    <div class="event-log">
      ${_eventLog.map(e => `<div class="event-item ${safeText(e.kind)}">[${safeText(e.ts)}] ${safeText(e.msg)}</div>`).join("")}
    </div>`;
}

function showDistanceModal(routeId) {
  if (!routeId) return;
  const label = document.getElementById("distanceRouteLabel");
  const input = document.getElementById("distanceInput");
  if (label) label.textContent = routeId;
  if (input) input.value = "";
  document.getElementById("distanceModal").classList.add("show");
  document.getElementById("distanceModal").dataset.routeId = routeId;
}

function hideDistanceModal() {
  document.getElementById("distanceModal").classList.remove("show");
}

async function saveDistance() {
  const modal = document.getElementById("distanceModal");
  const routeId = modal.dataset.routeId;
  if (!routeId) { hideDistanceModal(); return; }
  const raw = document.getElementById("distanceInput").value;
  const distance_m = raw === "" ? 0 : Number(raw);
  try {
    await fetch("/routes/" + encodeURIComponent(routeId) + "/distance" + qp, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({distance_m: Number.isFinite(distance_m) ? distance_m : 0})
    });
  } catch (e) {}
  hideDistanceModal();
  refreshRoutes();
  refreshAvgSpeed();
}

document.getElementById("distanceSave").onclick = saveDistance;
document.getElementById("distanceSkip").onclick = hideDistanceModal;
document.getElementById("distanceModal").addEventListener("click", (e) => {
  if (e.target.id === "distanceModal") hideDistanceModal();
});

let lastRouteId;
let _polling = false;
let _knownRouteIds = null;
const _promptedRouteIds = new Set();
async function pollStatus() {
  if (document.hidden) return;  // skip polling while tab is backgrounded
  if (_polling) return;         // guard: don't stack requests if a poll is slow
  _polling = true;
  try {
    const r = await fetch("/route/script/status" + qp);
    if (r.ok) {
      const j = await r.json();
      const st = j.status || {};
      const wasRunning = isRunning;
      isRunning = !!st.running;
      applyRecordState(j.record);
      currentRunningStep = st.current_step || 0;
      if (st.running) {
        const total = st.total || 0;
        const stepTiles = st.step && st.step.tiles != null ? Number(st.step.tiles) : null;
        const stepDeg = st.step && st.step.degrees != null ? Number(st.step.degrees) : null;
        const stepCount = stepTiles != null ? stepTiles : stepDeg;
        const stepDur = stepCount != null ? stepCount : ((st.step && Number(st.step.duration_s)) || 0);
        const stepElapsed = stepTiles != null ? Number(st.step_tiles_done || 0)
          : (stepDeg != null ? Number(st.step_degrees_done || 0) : Number(st.step_elapsed_s ?? st.elapsed_in_step_s ?? 0));
        const fillRatio = stepDur > 0 ? Math.min(1, stepElapsed / stepDur) : 0;
        const unit = stepTiles != null ? " ô" : (stepDeg != null ? "°" : "s");
        setPill(st.paused ? "pill-paused" : "pill-running", `${st.paused ? "paused" : "running"} ${currentRunningStep}/${total}`);
        const action = st.step ? safeText(st.step.action) : "";
        const pauseReason = st.pause_reason === "object_detected"
          ? "object"
          : (st.pause_reason === "manual_override" ? "manual" : (st.pause_reason || "script"));
        runDetail.textContent = action
          ? `${action} · ${stepTiles != null ? stepElapsed : stepElapsed.toFixed(1)} / ${stepTiles != null ? stepDur : stepDur.toFixed(1)}${unit}`
          : "";
        if (st.paused && action) {
          runDetail.textContent = `${action} - paused ${safeText(pauseReason)} - ${stepElapsed.toFixed(1)} / ${stepDur.toFixed(1)}${unit}`;
        }
        const overallRatio = total ? ((currentRunningStep - 1 + fillRatio) / total) : 0;
        progressBar.style.width = (overallRatio * 100).toFixed(1) + "%";
        renderProgressSegments(total, currentRunningStep, fillRatio);
      } else if (st.last_error) {
        setPill("pill-error", "error");
        runDetail.textContent = st.last_error;
        progressBar.style.width = "0%";
        renderProgressSegments(0, 0, 0);
      } else {
        if (isRecording) setPill("pill-running", "recording");
        else setPill("pill-idle", "idle");
        if (wasRunning) runDetail.textContent = "finished";
        progressBar.style.width = "0%";
        renderProgressSegments(0, 0, 0);
        currentRunningStep = 0;
      }
      if (wasRunning !== isRunning || st.running) render();
      if (wasRunning && !isRunning) {
        setTimeout(() => { refreshRoutes(); reloadStream(); }, 800);
      }
    }
  } catch (e) {}
  try {
    const r2 = await fetch(STATUS + qp);
    if (r2.ok) {
      const j2 = await r2.json();
      const tel = j2.telemetry || {};
      const rpi = j2.rpi_status || null;
      const rpiPayload = (rpi && rpi.payload) || {};
      const actuator = j2.actuator || null;

      renderStatusBar(rpi, actuator);
      renderMetricGrid(tel, rpiPayload, actuator);
      pushTelemetryHistory(tel);
      renderTrendChart();
      renderFsmStrip();
      renderHealthBanner(rpi);
      renderManualStatus(tel);

      const newRid = tel.route_id || null;
      if (lastRouteId !== undefined && lastRouteId !== newRid) {
        setTimeout(() => { refreshRoutes(); reloadStream(); }, 600);
      }
      lastRouteId = newRid;
    }
  } catch (e) {}
  finally { _polling = false; }
}

function reloadStream() {
  const img = document.getElementById("stream");
  if (!img) return;
  const base = STREAM + qp + (qp ? "&" : "?") + "_t=" + Date.now();
  img.src = base;
}
setInterval(pollStatus, 500);
render();
renderStatusBar(null);
renderMetricGrid({}, null);
renderEventLog();

const routesTbody = document.querySelector("#routesTable tbody");
const routeFilterChips = document.getElementById("routeFilterChips");
const routeSortSel = document.getElementById("routeSort");
const routeSearchInput = document.getElementById("routeSearch");
const routeSelectAll = document.getElementById("routeSelectAll");
const deleteSelectedBtn = document.getElementById("deleteSelected");
const compareSelectedBtn = document.getElementById("compareSelected");
let _routesAll = [];
let _routeFilter = "all";
const _selectedRoutes = new Set();
function fmtBytes(n) {
  if (n == null) return "-";
  if (n < 1024) return n + " B";
  if (n < 1024*1024) return (n/1024).toFixed(1) + " KB";
  return (n/1024/1024).toFixed(1) + " MB";
}
function fmtElapsed(s) {
  if (s == null) return "-";
  return Number(s).toFixed(1) + " s";
}
function fmtTs(t) {
  if (!t) return "-";
  return t.replace("T", " ").replace(/\.[0-9]+/, "").replace("+00:00", "Z");
}
async function refreshRoutes() {
  try {
    const r = await fetch("/routes/list?limit=50" + (TOKEN ? ("&token=" + encodeURIComponent(TOKEN)) : ""));
    if (!r.ok) return;
    const j = await r.json();
    _routesAll = j.routes || [];
    detectFinishedRoute(_routesAll);
    renderRoutes();
  } catch (e) {}
}

// Fires the distance-entry modal when a route's status transitions from
// "not_recorded" (its directory exists but route_summary.json hasn't been
// written yet -- true from the moment the route STARTS, not just while it
// runs) to an actual finished status. Tracking presence alone missed every
// route that outlived one 5s poll cycle, since its id was already known
// while it was still running.
function detectFinishedRoute(routes) {
  const nextStatus = new Map(routes.map(r => [r.route_id, r.status]));
  if (_knownRouteIds === null) {
    _knownRouteIds = nextStatus;
    return;
  }
  for (const rr of routes) {
    const prevStatus = _knownRouteIds.get(rr.route_id);
    const wasFinished = !!prevStatus && prevStatus !== "not_recorded";
    const isFinished = !!rr.status && rr.status !== "not_recorded";
    if (isFinished && !wasFinished && !_promptedRouteIds.has(rr.route_id)) {
      _promptedRouteIds.add(rr.route_id);
      showDistanceModal(rr.route_id);
    }
  }
  _knownRouteIds = nextStatus;
}

function renderRoutes() {
  const filtered = applyRouteFilters(_routesAll);
  routesTbody.innerHTML = "";
  filtered.forEach(rr => {
    const tr = document.createElement("tr");
    tr.className = "route-clickable";
    tr.dataset.name = rr.route_id;
    tr.appendChild(makeCheckCell(rr.route_id));
    tr.appendChild(makeTextCell(rr.route_id));
    tr.appendChild(makeTextCell(rr.route_mode || '-'));
    tr.appendChild(makePresetCell(rr));
    tr.appendChild(makeStatusCell(rr));
    tr.appendChild(makeTextCell(rr.distance_m != null ? Number(rr.distance_m).toFixed(3) + " m" : '-'));
    tr.appendChild(makeTextCell(fmtElapsed(rr.elapsed)));
    tr.appendChild(makeTextCell(fmtBytes(rr.zip_size)));
    tr.appendChild(makeTextCell(fmtTs(rr.ended_utc)));
    tr.appendChild(makeActionsCell(rr));
    routesTbody.appendChild(tr);
  });
  updateSelectionUI();
}

function makeTextCell(text) {
  const td = document.createElement("td");
  td.textContent = text;
  return td;
}
function makeCheckCell(routeId) {
  const td = document.createElement("td");
  const cb = document.createElement("input");
  cb.type = "checkbox"; cb.className = "route-pick";
  cb.dataset.name = routeId;
  cb.checked = _selectedRoutes.has(routeId);
  cb.addEventListener("click", e => e.stopPropagation());
  td.appendChild(cb);
  return td;
}
function makePresetCell(rr) {
  const td = document.createElement("td");
  if (rr.preset) {
    const span = document.createElement("span");
    span.className = "badge badge-ok";
    span.textContent = rr.preset;
    td.appendChild(span);
  } else if (rr.script_source) {
    const span = document.createElement("span");
    span.className = "muted";
    span.textContent = rr.script_source;
    td.appendChild(span);
  } else {
    const span = document.createElement("span");
    span.className = "muted";
    span.textContent = "-";
    td.appendChild(span);
  }
  return td;
}
function makeStatusCell(rr) {
  const td = document.createElement("td");
  const accepted = rr.accepted === false ? " ✗" : (rr.accepted === true ? " ✓" : "");
  td.textContent = (rr.status || '-') + accepted;
  return td;
}
function makeActionsCell(rr) {
  const td = document.createElement("td");
  if (rr.has_zip) {
    const a = document.createElement("a");
    a.className = "pill pill-running";
    a.style.cssText = "text-decoration:none;padding:4px 10px;margin-right:6px";
    a.href = `/routes/download/${encodeURIComponent(rr.route_id)}${qp}`;
    a.textContent = "⬇";
    a.addEventListener("click", e => e.stopPropagation());
    td.appendChild(a);
  } else {
    const span = document.createElement("span");
    span.className = "muted";
    span.style.marginRight = "6px";
    span.textContent = "no zip";
    td.appendChild(span);
  }
  const btn = document.createElement("button");
  btn.className = "rm route-del";
  btn.dataset.name = rr.route_id;
  btn.style.padding = "4px 10px";
  btn.textContent = "🗑";
  td.appendChild(btn);
  return td;
}

function applyRouteFilters(list) {
  let out = list.slice();
  if (_routeFilter === "accepted") out = out.filter(r => r.accepted === true);
  else if (_routeFilter === "rejected") out = out.filter(r => r.accepted === false);
  const q = (routeSearchInput && routeSearchInput.value || "").trim().toLowerCase();
  if (q) out = out.filter(r => (r.route_id || "").toLowerCase().includes(q));
  const sort = routeSortSel ? routeSortSel.value : "recent";
  if (sort === "longest") out.sort((a, b) => (b.elapsed || 0) - (a.elapsed || 0));
  else if (sort === "distance") out.sort((a, b) => (b.distance_m || 0) - (a.distance_m || 0));
  else out.sort((a, b) => (b.ended_utc || "").localeCompare(a.ended_utc || ""));
  return out;
}

function updateSelectionUI() {
  if (!deleteSelectedBtn) return;
  const n = _selectedRoutes.size;
  deleteSelectedBtn.disabled = n === 0;
  deleteSelectedBtn.textContent = n ? `🗑 selected (${n})` : "🗑 selected";
  if (compareSelectedBtn) {
    compareSelectedBtn.disabled = n < 2;
    compareSelectedBtn.textContent = n >= 2 ? `⊟ compare (${n})` : "⊟ compare";
  }
}

if (routeFilterChips) routeFilterChips.onclick = (e) => {
  if (!e.target.classList.contains("chip")) return;
  routeFilterChips.querySelectorAll(".chip").forEach(c => c.classList.toggle("active", c === e.target));
  _routeFilter = e.target.dataset.filter || "all";
  renderRoutes();
};
if (routeSortSel) routeSortSel.onchange = renderRoutes;
if (routeSearchInput) routeSearchInput.oninput = renderRoutes;
if (routeSelectAll) routeSelectAll.onchange = (e) => {
  const want = e.target.checked;
  applyRouteFilters(_routesAll).forEach(r => want ? _selectedRoutes.add(r.route_id) : _selectedRoutes.delete(r.route_id));
  renderRoutes();
};
if (deleteSelectedBtn) deleteSelectedBtn.onclick = async () => {
  const n = _selectedRoutes.size;
  if (!n) return;
  if (!confirm(`Delete ${n} selected route(s)? This removes directories + zips.`)) return;
  for (const id of Array.from(_selectedRoutes)) {
    try { await fetch(`/routes/${encodeURIComponent(id)}${qp}`, {method: "DELETE"}); } catch (e) {}
  }
  _selectedRoutes.clear();
  refreshRoutes();
};
const exportSelectedBtn = document.getElementById("exportSelected");
if (exportSelectedBtn) exportSelectedBtn.onclick = () => {
  const ids = Array.from(_selectedRoutes);
  const sep = qp ? "&" : "?";
  const idsParam = ids.length ? sep + "ids=" + encodeURIComponent(ids.join(",")) : "";
  window.location.href = "/routes/export.xlsx" + qp + idsParam;
};
const downloadAllBtn = document.getElementById("downloadAllSelected");
if (downloadAllBtn) downloadAllBtn.onclick = () => {
  const ids = Array.from(_selectedRoutes);
  const sep = qp ? "&" : "?";
  const idsParam = ids.length ? sep + "ids=" + encodeURIComponent(ids.join(",")) : "";
  window.location.href = "/routes/download_all" + qp + idsParam;
};
document.getElementById("refreshRoutes").onclick = refreshRoutes;
setInterval(refreshRoutes, 5000);
refreshRoutes();

routesTbody.onclick = async (e) => {
  const t = e.target;
  if (t.classList.contains("route-del")) {
    const name = t.dataset.name;
    if (!confirm(`Delete route ${name}? This removes its directory and zip.`)) return;
    const r = await fetch(`/routes/${encodeURIComponent(name)}${qp}`, {method: "DELETE"});
    if (!r.ok) { alert("delete failed"); return; }
    refreshRoutes();
    return;
  }
  const tr = t.closest("tr.route-clickable");
  if (tr && tr.dataset.name) {
    openSummary(tr.dataset.name);
  }
};

async function openSummary(name) {
  const overview = document.getElementById("summaryOverview");
  const scriptDiv = document.getElementById("summaryScript");
  const jsonPre = document.getElementById("summaryJson");
  const dlBtn = document.getElementById("summaryDownload");
  const title = document.getElementById("summaryTitle");
  overview.innerHTML = `<div class="muted" style="margin-top:14px">loading…</div>`;
  scriptDiv.innerHTML = "";
  jsonPre.textContent = "";
  if (dlBtn) dlBtn.style.display = "none";
  title.textContent = `Route summary · ${name}`;
  document.getElementById("summaryModal").classList.add("show");
  // reset to Overview tab on open
  document.querySelectorAll(".modal-tabs .tab").forEach(t => t.classList.toggle("active", t.dataset.mtab === "overview"));
  document.querySelectorAll(".tab-pane[data-mpane]").forEach(p => p.classList.toggle("active", p.dataset.mpane === "overview"));
  try {
    const r = await fetch(`/routes/${encodeURIComponent(name)}/summary${qp}`);
    if (!r.ok) {
      overview.innerHTML = `<div class="muted" style="margin-top:14px;color:var(--bad)">load failed (${r.status})</div>`;
      return;
    }
    const j = await r.json();
    const s = j.summary || {};
    overview.innerHTML = renderSummaryOverview(s);
    scriptDiv.innerHTML = renderSummaryScript(s);
    jsonPre.textContent = JSON.stringify(s, null, 2);
    if (dlBtn) {
      const matched = _routesAll.find(rr => rr.route_id === name);
      if (matched && matched.has_zip) {
        dlBtn.href = `/routes/download/${encodeURIComponent(name)}${qp}`;
        dlBtn.style.display = "";
      }
    }
  } catch (e) {
    overview.innerHTML = `<div class="muted" style="margin-top:14px;color:var(--bad)">network error</div>`;
  }
}

function renderSummaryOverview(s) {
  const accepted = s.accepted === true ? '<span class="badge badge-ok">accepted ✓</span>'
    : (s.accepted === false ? '<span class="badge badge-fail">rejected ✗</span>' : '-');
  return `
    <div class="kv">
      <div class="k">Route ID</div><div class="v">${safeText(s.route_id)}</div>
      <div class="k">Mode</div><div class="v">${safeText(s.route_mode)}</div>
      <div class="k">Tune profile</div><div class="v">${safeText((s.extra_meta || {}).tune_profile)}</div>
      <div class="k">Status</div><div class="v">${safeText(s.status)} ${accepted}</div>
      <div class="k">Rejection reason</div><div class="v">${safeText(s.rejection_reason)}</div>
      <div class="k">Started (UTC)</div><div class="v">${safeText(s.start_timestamp_utc)}</div>
      <div class="k">Ended (UTC)</div><div class="v">${safeText(s.end_timestamp_utc)}</div>
      <div class="k">Elapsed</div><div class="v">${s.total_elapsed_seconds != null ? Number(s.total_elapsed_seconds).toFixed(2) + ' s' : '-'}</div>
      <div class="k">Total frames</div><div class="v">${s.total_frames ?? '-'}</div>
      <div class="k">Frames with theta</div><div class="v">${s.frames_with_theta ?? '-'}</div>
      <div class="k">Gap ratio</div><div class="v">${s.gap_ratio != null ? Number(s.gap_ratio).toFixed(3) : '-'}</div>
      <div class="k">HW errors</div><div class="v">${s.hardware_error_count ?? '-'}</div>
      <div class="k">Abstract steps</div><div class="v">${s.abstract_steps ?? '-'}</div>
    </div>`;
}

function renderSummaryScript(s) {
  const extra = s.extra_meta || {};
  const script = extra.script || {};
  const stepsRows = (script.steps || []).map((st, i) => {
    const d = Number(st.duration_s);
    const dTxt = st.tiles != null ? `${st.tiles} ô` : (st.degrees != null ? `${Number(st.degrees).toFixed(0)}°` : (Number.isFinite(d) ? d.toFixed(1) + " s" : "-"));
    return `<tr><td>${i+1}</td><td>${safeText(st.action)}</td><td>${dTxt}</td></tr>`;
  }).join("");
  const stepsTable = stepsRows
    ? `<table style="margin-top:6px"><thead><tr><th>#</th><th>Action</th><th>Duration</th></tr></thead><tbody>${stepsRows}</tbody></table>`
    : `<div class="muted">no script steps recorded</div>`;
  let submittedAt = null;
  if (script.submitted_at_unix) {
    const ms = Number(script.submitted_at_unix) * 1000;
    if (Number.isFinite(ms)) {
      try { submittedAt = new Date(ms).toISOString(); } catch (e) { submittedAt = null; }
    }
  }
  return `
    <div class="kv">
      <div class="k">Source</div><div class="v">${safeText(script.source)}</div>
      <div class="k">Preset name</div><div class="v">${safeText(script.preset_name)}</div>
      <div class="k">Description</div><div class="v">${safeText(script.description)}</div>
      <div class="k">Submitted (UTC)</div><div class="v">${safeText(submittedAt)}</div>
      <div class="k">Step count</div><div class="v">${(script.steps || []).length}</div>
    </div>
    <h3>Steps</h3>
    ${stepsTable}`;
}

// modal tabs handler
document.querySelectorAll(".modal-tabs .tab").forEach(btn => {
  btn.onclick = () => {
    const target = btn.dataset.mtab;
    document.querySelectorAll(".modal-tabs .tab").forEach(t => t.classList.toggle("active", t.dataset.mtab === target));
    document.querySelectorAll(".tab-pane[data-mpane]").forEach(p => p.classList.toggle("active", p.dataset.mpane === target));
  };
});

document.getElementById("summaryClose").onclick = () => document.getElementById("summaryModal").classList.remove("show");
document.getElementById("summaryModal").addEventListener("click", (e) => {
  if (e.target.id === "summaryModal") e.currentTarget.classList.remove("show");
});

document.getElementById("deleteAllRoutes").onclick = async () => {
  if (!confirm("Delete ALL routes (directories + zips)? This cannot be undone.")) return;
  if (!confirm("Are you really sure? This wipes recorded data.")) return;
  const r = await fetch("/routes/delete_all" + qp, {method: "POST"});
  const j = await r.json().catch(() => ({}));
  alert(`removed=${j.removed||0} errors=${(j.errors||[]).length}`);
  refreshRoutes();
};

// ---------- presets CRUD ----------
const presetSelect = document.getElementById("presetSelect");
const presetName = document.getElementById("presetName");
async function refreshPresets() {
  try {
    const r = await fetch("/presets" + qp);
    if (!r.ok) return;
    const j = await r.json();
    const list = j.presets || [];
    const cur = presetSelect.value;
    presetSelect.innerHTML = "<option value=\"\">— select preset —</option>";
    list.forEach(p => {
      const o = document.createElement("option");
      o.value = p.name;
      o.textContent = `${p.name} (${p.steps_count} steps)`;
      presetSelect.appendChild(o);
    });
    if (cur) presetSelect.value = cur;
    Object.entries(turnPresetSel).forEach(([side, sel]) => {
      if (!sel || document.activeElement === sel) return;
      const names = list.map(p => p.name);
      if (turnPresetChoice[side] && !names.includes(turnPresetChoice[side])) names.push(turnPresetChoice[side]);
      sel.innerHTML = `<option value="">mặc định</option>` + names
        .map(n => `<option value="${safeText(n)}">${safeText(n)}</option>`).join("");
      sel.value = turnPresetChoice[side] || "";
    });
  } catch (e) {}
}
document.getElementById("presetLoad").onclick = async () => {
  const name = presetSelect.value;
  if (!name) { alert("select a preset first"); return; }
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`);
  if (!r.ok) { alert("load failed"); return; }
  const j = await r.json();
  const p = j.preset || {};
  steps.length = 0;
  (p.steps || []).forEach(s => steps.push(presetStep(s)));
  presetName.value = p.name || name;
  resetEditor();
  render();
};
document.getElementById("presetSave").onclick = async () => {
  const name = (presetName.value || "").trim();
  if (!name) { alert("enter preset name"); return; }
  if (steps.length === 0) { alert("add steps first"); return; }
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`, {method: "PUT", headers: {"Content-Type": "application/json"}, body: JSON.stringify({steps})});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { alert("save failed: " + JSON.stringify(j.detail || r.status)); return; }
  await refreshPresets();
  presetSelect.value = name;
};
document.getElementById("presetDelete").onclick = async () => {
  const name = presetSelect.value;
  if (!name) { alert("select a preset first"); return; }
  if (!confirm(`Delete preset "${name}"?`)) return;
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`, {method: "DELETE"});
  if (!r.ok) { alert("delete failed"); return; }
  presetSelect.value = "";
  refreshPresets();
};
refreshPresets();
setInterval(refreshPresets, 10000);

// ---------- tune panel ----------
const tuneFields = document.getElementById("tuneFields");
const tuneStatus = document.getElementById("tuneStatus");
const tuneApplyBtn = document.getElementById("tuneApply");
const tuneSaveBtn = document.getElementById("tuneSave");
const tuneResetSavedBtn = document.getElementById("tuneResetSaved");
const tuneResetDefaultsBtn = document.getElementById("tuneResetDefaults");
let tuneState = null;
let tuneDraft = {};
let tuneTouched = false;
let tuneEditing = false;
let tuneRenderedSchema = "";
// Group filter: "" = ALL. Remembered per browser; hidden groups keep their staged values.
const tuneGroupFilter = document.getElementById("tuneGroupFilter");
let tuneGroup = "";
try { tuneGroup = localStorage.getItem("tuneGroup") || ""; } catch (e) { tuneGroup = ""; }

function renderTuneGroupOptions(groups) {
  if (!tuneGroupFilter) return;
  if (tuneGroup && !groups.includes(tuneGroup)) tuneGroup = "";
  tuneGroupFilter.innerHTML = `<option value="">ALL</option>` + groups
    .map(g => `<option value="${safeText(g)}">${safeText(g)}</option>`).join("");
  tuneGroupFilter.value = tuneGroup;
}

function tuneSchemaSig(schema) {
  return (schema || []).map(s => s.key).join(",");
}

function tuneSame(a, b, schema) {
  if (!a || !b || !schema) return false;
  return schema.every(s => Math.abs(Number(a[s.key]) - Number(b[s.key])) <= 1e-9);
}

function tuneFmt(value, spec) {
  const n = Number(value);
  if (!Number.isFinite(n)) return "-";
  return spec.type === "int" ? String(Math.round(n)) : n.toFixed(spec.step < 0.1 ? 2 : 1);
}

function tuneLocalDirty() {
  return !!(tuneState && tuneTouched && !tuneSame(tuneDraft, tuneState.values, tuneState.schema));
}

function renderTuneStatus() {
  if (!tuneStatus || !tuneState) return;
  const idle = tuneState.idle || {};
  const saved = tuneState.saved || {};
  const localDirty = tuneLocalDirty();
  const parts = [
    idle.ok ? "idle" : safeText(idle.reason || "blocked"),
    localDirty ? "staged changes" : (tuneState.dirty ? "unsaved active values" : "active saved/default"),
    saved.exists ? `saved ${safeText(saved.updated_utc || "")}` : "not saved",
  ];
  const profile = (tuneState.profile || {}).active;
  if (profile) parts.unshift(`profile ${safeText(profile)}`);
  tuneStatus.textContent = parts.filter(Boolean).join(" · ");
  renderTuneProfiles();
  if (tuneApplyBtn) tuneApplyBtn.disabled = !idle.ok || !localDirty;
  if (tuneSaveBtn) tuneSaveBtn.disabled = localDirty;
  if (tuneResetSavedBtn) tuneResetSavedBtn.disabled = !idle.ok || !saved.exists;
  if (tuneResetDefaultsBtn) tuneResetDefaultsBtn.disabled = !idle.ok;
}

function renderTuneFields() {
  if (!tuneFields || !tuneState) return;
  const byGroup = {};
  (tuneState.schema || []).forEach(spec => {
    if (!byGroup[spec.group]) byGroup[spec.group] = [];
    byGroup[spec.group].push(spec);
  });
  renderTuneGroupOptions(Object.keys(byGroup));
  let html = "";
  Object.keys(byGroup).filter(group => !tuneGroup || group === tuneGroup).forEach(group => {
    html += `<h3>${safeText(group)}</h3>`;
    byGroup[group].forEach(spec => {
      const key = spec.key;
      const value = tuneDraft[key] ?? tuneState.values[key];
      html += `
        <div class="tune-field" data-key="${safeText(key)}">
          <label for="tune_${safeText(key)}">${safeText(spec.label)}</label>
          <input id="tune_${safeText(key)}_range" data-tune-input="1" data-key="${safeText(key)}" type="range" min="${spec.min}" max="${spec.max}" step="${spec.step}" value="${safeText(value)}">
          <input id="tune_${safeText(key)}" data-tune-input="1" data-key="${safeText(key)}" type="number" min="${spec.min}" max="${spec.max}" step="${spec.step}" value="${safeText(value)}">
          <div class="tune-meta">current ${safeText(tuneFmt(tuneState.values[key], spec))} · default ${safeText(tuneFmt(tuneState.defaults[key], spec))}</div>
        </div>`;
    });
  });
  tuneFields.innerHTML = html;
  tuneRenderedSchema = tuneSchemaSig(tuneState.schema);
  renderTuneStatus();
}

function tuneInputFocused() {
  return !!(tuneFields && document.activeElement && tuneFields.contains(document.activeElement));
}

async function refreshTune(forceRender) {
  if (!tuneFields) return;
  try {
    const r = await fetch("/api/tune" + qp);
    if (!r.ok) {
      if (tuneStatus) tuneStatus.textContent = "tune unavailable";
      return;
    }
    const data = await r.json();
    tuneState = data;
    // Re-render when the server's param list changes (e.g. after a redeploy
    // while this tab stayed open), not only on first load.
    const shouldRenderFields = forceRender || !tuneFields.hasChildNodes()
      || tuneSchemaSig(data.schema) !== tuneRenderedSchema;
    if (shouldRenderFields && !tuneEditing && !tuneInputFocused()) {
      tuneDraft = Object.assign({}, data.values || {});
      tuneTouched = false;
      renderTuneFields();
    } else {
      renderTuneStatus();
    }
  } catch (e) {
    if (tuneStatus) tuneStatus.textContent = "tune network error";
  }
}

function updateTunePair(key, value) {
  document.querySelectorAll(`[data-tune-input][data-key="${key}"]`).forEach(input => {
    if (input === document.activeElement && input.type === "number") return;
    input.value = String(value);
  });
}

async function tunePost(path, body, method) {
  try {
    const r = await fetch(path + qp, {
      method: method || "POST",
      headers: {"Content-Type": "application/json"},
      body: body == null ? undefined : JSON.stringify(body),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) {
      if (tuneStatus) tuneStatus.textContent = "tune error: " + (j.detail || r.status);
      return;
    }
    tuneState = j;
    tuneDraft = Object.assign({}, j.values || {});
    tuneTouched = false;
    renderTuneFields();
  } catch (e) {
    if (tuneStatus) tuneStatus.textContent = "tune network error";
  }
}

if (tuneFields) {
  tuneFields.addEventListener("focusin", e => {
    if (e.target && e.target.dataset && e.target.dataset.tuneInput) tuneEditing = true;
  });
  tuneFields.addEventListener("focusout", () => {
    setTimeout(() => {
      tuneEditing = !!tuneInputFocused();
    }, 0);
  });
  tuneFields.addEventListener("input", e => {
    const input = e.target;
    if (!input || !input.dataset || !input.dataset.key || !tuneState) return;
    const key = input.dataset.key;
    const spec = (tuneState.schema || []).find(s => s.key === key);
    if (!spec) return;
    const n = Number(input.value);
    if (!Number.isFinite(n)) return;
    const value = spec.type === "int" ? Math.round(n) : n;
    tuneDraft[key] = value;
    tuneTouched = true;
    updateTunePair(key, value);
    renderTuneStatus();
  });
}
if (tuneGroupFilter) {
  tuneGroupFilter.onchange = () => {
    tuneGroup = tuneGroupFilter.value;
    try { localStorage.setItem("tuneGroup", tuneGroup); } catch (e) { /* private window */ }
    renderTuneFields();
  };
}
// ---------- tune profiles (named calib_tune sets) ----------
const tuneProfileSelect = document.getElementById("tuneProfileSelect");
const tuneProfileMenu = document.querySelector(".tune-profile-menu");
let tuneProfileSig = "";

function renderTuneProfiles() {
  if (!tuneProfileSelect || !tuneState || !tuneState.profile) return;
  const prof = tuneState.profile;
  const list = prof.list || [];
  const sig = list.map(p => `${p.name}:${p.updated_utc || ""}`).join("|") + "@" + prof.active;
  const idle = !!(tuneState.idle || {}).ok;
  tuneProfileSelect.disabled = !idle;
  if (sig === tuneProfileSig || document.activeElement === tuneProfileSelect) return;
  tuneProfileSig = sig;
  tuneProfileSelect.innerHTML = list.map(p => {
    const note = p.saved ? "" : " (chưa lưu)";
    return `<option value="${safeText(p.name)}">${safeText(p.name)}${note}</option>`;
  }).join("");
  tuneProfileSelect.value = prof.active;
}

function closeTuneProfileMenu() { if (tuneProfileMenu) tuneProfileMenu.open = false; }

if (tuneProfileSelect) {
  tuneProfileSelect.onchange = async () => {
    const name = tuneProfileSelect.value;
    const active = ((tuneState || {}).profile || {}).active;
    if (!name || name === active) return;
    const unsaved = tuneLocalDirty() || (tuneState && tuneState.dirty);
    if (unsaved && !confirm(`Giá trị hiện tại chưa Save vào "${active}" sẽ bị thay bằng "${name}". Tiếp tục?`)) {
      tuneProfileSelect.value = active;
      return;
    }
    tuneProfileSig = "";
    await tunePost("/api/tune/profile/load", {name});
  };
}
const tuneProfileNewBtn = document.getElementById("tuneProfileNew");
if (tuneProfileNewBtn) tuneProfileNewBtn.onclick = async () => {
  closeTuneProfileMenu();
  if (tuneLocalDirty()) { tuneStatus.textContent = "Apply hoặc bỏ thay đổi đang sửa trước"; return; }
  const name = (prompt("Tên profile mới (chữ, số, - hoặc _):") || "").trim();
  if (!name) return;
  tuneProfileSig = "";
  await tunePost("/api/tune/profile/create", {name});
};
const tuneProfileRenameBtn = document.getElementById("tuneProfileRename");
if (tuneProfileRenameBtn) tuneProfileRenameBtn.onclick = async () => {
  closeTuneProfileMenu();
  const active = ((tuneState || {}).profile || {}).active;
  if (!active || active === "default") { tuneStatus.textContent = "không đổi tên được profile default"; return; }
  const name = (prompt(`Đổi tên "${active}" thành:`, active) || "").trim();
  if (!name || name === active) return;
  tuneProfileSig = "";
  await tunePost("/api/tune/profile/rename", {name: active, new_name: name});
};
const tuneProfileDeleteBtn = document.getElementById("tuneProfileDelete");
if (tuneProfileDeleteBtn) tuneProfileDeleteBtn.onclick = async () => {
  closeTuneProfileMenu();
  const prof = (tuneState || {}).profile || {};
  const others = (prof.list || []).filter(p => p.name !== "default" && p.name !== prof.active).map(p => p.name);
  if (!others.length) { tuneStatus.textContent = "không có profile nào xoá được (không xoá default hoặc profile đang dùng)"; return; }
  const name = (prompt(`Xoá profile nào? (${others.join(", ")})\nKhông xoá được default hoặc profile đang dùng.`) || "").trim();
  if (!name) return;
  if (!others.includes(name)) { tuneStatus.textContent = `không xoá được "${name}"`; return; }
  if (!confirm(`Xoá hẳn profile "${name}"? Không hoàn tác được.`)) return;
  tuneProfileSig = "";
  await tunePost("/api/tune/profile/delete", {name});
};
document.addEventListener("click", e => {
  if (tuneProfileMenu && tuneProfileMenu.open && !tuneProfileMenu.contains(e.target)) closeTuneProfileMenu();
});

if (tuneApplyBtn) tuneApplyBtn.onclick = () => tunePost("/api/tune", {values: tuneDraft}, "PUT");
if (tuneSaveBtn) tuneSaveBtn.onclick = () => tunePost("/api/tune/save");
if (tuneResetSavedBtn) tuneResetSavedBtn.onclick = () => tunePost("/api/tune/reset", {target: "saved"});
if (tuneResetDefaultsBtn) tuneResetDefaultsBtn.onclick = () => tunePost("/api/tune/reset", {target: "defaults"});
refreshTune(true);

// Calibration tab wizards (Fit ground k, c and Turn suggestion) live in §calib below.
setInterval(() => refreshTune(false), 1000);

// §13 ─ Telemetry trends (F1 chart), FSM strip (F2), health banner (F3) ──
const _telHistory = [];           // {t, theta, servo, fsm}
const TEL_HISTORY_MS = 30000;     // keep last 30s
const FSM_COLORS = {
  GAPPING: "#6b6f78",
  DANGER_LEFT: "#f87171",
  DANGER_RIGHT: "#fbbf24",
  TRACKING_COAST: "#60a5fa",
  TRACKING_PD: "#4ade80",
};

function pushTelemetryHistory(tel) {
  const now = Date.now();
  _telHistory.push({
    t: now,
    theta: (tel && tel.theta != null) ? Number(tel.theta) : null,
    servo: (tel && tel.servo_angle != null) ? Number(tel.servo_angle) : null,
    fsm: (tel && tel.fsm_state) || null,
  });
  const cutoff = now - TEL_HISTORY_MS;
  while (_telHistory.length && _telHistory[0].t < cutoff) _telHistory.shift();
}

function _sparkPath(points, getY, x0, span, yMin, yMax, h) {
  if (points.length < 2 || yMax === yMin) return "";
  return points.map((p, i) => {
    const x = ((p.t - x0) / span) * 100;
    const y = h - ((getY(p) - yMin) / (yMax - yMin)) * h;
    return `${i === 0 ? "M" : "L"}${x.toFixed(2)},${y.toFixed(2)}`;
  }).join(" ");
}

function renderTrendChart() {
  const el = document.getElementById("trendChart");
  if (!el) return;
  const pts = _telHistory.filter(p => p.theta != null || p.servo != null);
  if (pts.length < 2) { el.innerHTML = '<div class="muted" style="font-size:11px">collecting…</div>'; return; }
  const x0 = pts[0].t;
  const span = Math.max(1, pts[pts.length - 1].t - x0);
  const h = 48;
  const thetaPts = pts.filter(p => p.theta != null);
  const servoPts = pts.filter(p => p.servo != null);
  // shared Y scale across both series so they're comparable
  const allY = pts.flatMap(p => [p.theta, p.servo]).filter(v => v != null);
  const yMin = Math.min(...allY), yMax = Math.max(...allY);
  const thetaPath = _sparkPath(thetaPts, p => p.theta, x0, span, yMin, yMax, h);
  const servoPath = _sparkPath(servoPts, p => p.servo, x0, span, yMin, yMax, h);
  el.innerHTML = `
    <div class="metric-section-label">Theta / Servo (last 30s)</div>
    <svg viewBox="0 0 100 ${h}" preserveAspectRatio="none" class="trend-svg">
      <path d="${servoPath}" fill="none" stroke="var(--warn)" stroke-width="0.8" vector-effect="non-scaling-stroke"/>
      <path d="${thetaPath}" fill="none" stroke="var(--info)" stroke-width="0.8" vector-effect="non-scaling-stroke"/>
    </svg>
    <div class="trend-legend"><span class="text-warn">■ servo</span> <span style="color:var(--info)">■ theta</span> <span class="muted">${yMin.toFixed(0)}…${yMax.toFixed(0)}°</span></div>`;
}

function renderFsmStrip() {
  const el = document.getElementById("fsmStrip");
  if (!el) return;
  const pts = _telHistory.filter(p => p.fsm);
  if (!pts.length) { el.innerHTML = ""; return; }
  const x0 = pts[0].t;
  const span = Math.max(1, Date.now() - x0);
  let segs = "";
  for (let i = 0; i < pts.length; i++) {
    const start = pts[i].t;
    const end = (i + 1 < pts.length) ? pts[i + 1].t : Date.now();
    const left = ((start - x0) / span) * 100;
    const w = ((end - start) / span) * 100;
    const color = FSM_COLORS[pts[i].fsm] || "#444";
    segs += `<div class="fsm-seg" style="left:${left.toFixed(2)}%;width:${w.toFixed(2)}%;background:${color}" title="${safeText(pts[i].fsm)}"></div>`;
  }
  const cur = pts[pts.length - 1].fsm;
  el.innerHTML = `
    <div class="metric-section-label">FSM state (last 30s) · now: <span style="color:${FSM_COLORS[cur] || '#888'}">${safeText(cur)}</span></div>
    <div class="fsm-track">${segs}</div>`;
}

function renderHealthBanner(rpi) {
  const el = document.getElementById("healthBanner");
  if (!el) return;
  const payload = (rpi && rpi.payload) || {};
  const stale = !rpi || !!rpi.stale;
  const estop = !!payload.estop_active;
  const objectNear = !!payload.object_pause_active || payload.object_state === "near";
  const objectError = payload.object_detector_error || "";
  if (estop) {
    el.className = "health-banner show bad";
    el.innerHTML = "";
    const span = document.createElement("span");
    span.textContent = "⛔ E-STOP ACTIVE — vehicle latched safe. ";
    const btn = document.createElement("button");
    btn.textContent = "Reset E-stop";
    btn.className = "estop-reset-btn";
    btn.onclick = estopReset;
    el.appendChild(span);
    el.appendChild(btn);
  } else if (objectNear) {
    el.className = "health-banner show bad";
    el.textContent = "Object near - route paused, base stopped, servo released.";
  } else if (objectError) {
    el.className = "health-banner show warn";
    el.textContent = "Object detector error - " + objectError;
  } else if (stale) {
    el.className = "health-banner show warn";
    el.textContent = "⚠ Actuator telemetry stale — MQTT peer offline or connection lost.";
  } else {
    el.className = "health-banner";
    el.textContent = "";
  }
}

async function estopReset() {
  if (!confirm("Reset E-stop? This only clears if the physical button is released (safe).")) return;
  try {
    const r = await fetch("/control/estop_reset" + qp, {method: "POST"});
    if (!r.ok) { alert("reset request failed: " + r.status); return; }
    addEvent("info", "E-stop reset requested");
  } catch (e) {
    alert("reset network error");
  }
}

if (compareSelectedBtn) compareSelectedBtn.onclick = async () => {
  const ids = Array.from(_selectedRoutes).slice(0, 3);
  if (ids.length < 2) return;
  await openCompare(ids);
};

async function openCompare(ids) {
  const overview = document.getElementById("summaryOverview");
  const scriptDiv = document.getElementById("summaryScript");
  const jsonPre = document.getElementById("summaryJson");
  const dlBtn = document.getElementById("summaryDownload");
  const title = document.getElementById("summaryTitle");
  if (dlBtn) dlBtn.style.display = "none";
  title.textContent = `Compare ${ids.length} routes`;
  scriptDiv.innerHTML = '<div class="muted">comparison uses the Overview tab</div>';
  jsonPre.textContent = "";
  overview.innerHTML = '<div class="muted" style="margin-top:14px">loading…</div>';
  document.getElementById("summaryModal").classList.add("show");
  document.querySelectorAll(".modal-tabs .tab").forEach(t => t.classList.toggle("active", t.dataset.mtab === "overview"));
  document.querySelectorAll(".tab-pane[data-mpane]").forEach(p => p.classList.toggle("active", p.dataset.mpane === "overview"));
  try {
    const summaries = await Promise.all(ids.map(async id => {
      try {
        const r = await fetch(`/routes/${encodeURIComponent(id)}/summary${qp}`);
        if (!r.ok) return {route_id: id, _error: r.status};
        const j = await r.json();
        return j.summary || {route_id: id};
      } catch (e) { return {route_id: id, _error: "net"}; }
    }));
    overview.innerHTML = renderCompare(summaries);
    jsonPre.textContent = JSON.stringify(summaries, null, 2);
  } catch (e) {
    overview.innerHTML = '<div class="muted" style="margin-top:14px;color:var(--bad)">compare failed</div>';
  }
}

function renderCompare(summaries) {
  const rows = [
    ["Mode", s => safeText(s.route_mode)],
    ["Status", s => safeText(s.status)],
    ["Accepted", s => s.accepted === true ? "✓" : (s.accepted === false ? "✗" : "-")],
    ["Elapsed", s => s.total_elapsed_seconds != null ? Number(s.total_elapsed_seconds).toFixed(2) + " s" : "-"],
    ["Frames", s => s.total_frames ?? "-"],
    ["Frames w/ theta", s => s.frames_with_theta ?? "-"],
    ["Gap ratio", s => s.gap_ratio != null ? Number(s.gap_ratio).toFixed(3) : "-"],
    ["HW errors", s => s.hardware_error_count ?? "-"],
  ];
  const head = `<th>Metric</th>` + summaries.map(s => `<th>${safeText(s.route_id)}</th>`).join("");
  const body = rows.map(([label, fn]) =>
    `<tr><td class="cmp-metric">${label}</td>` + summaries.map(s => `<td>${s._error ? '<span class="text-bad">err</span>' : fn(s)}</td>`).join("") + `</tr>`
  ).join("");
  return `<table class="data-table cmp-table"><thead><tr>${head}</tr></thead><tbody>${body}</tbody></table>`;
}



// ---------------------------------------------------------------------------
// §13 Map tab. The editor follows the ARL_AUTOCAR map survey tool (sparse
// grid with rulers, pan/zoom/fit, draw/erase, measure, anchors, start/goal,
// route preview, save/load/delete); on top it shows the car's live cell.
// ---------------------------------------------------------------------------
function escapeHtml(v) {
  return String(v == null ? "" : v).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
}

function activeTab() {
  const t = document.querySelector(".tab.active");
  return t ? t.dataset.tab : "";
}

async function apiJson(path, body) {
  const opts = body === undefined ? {} : {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  };
  const sep = path.includes("?") ? (qp ? "&" + qp.slice(1) : "") : qp;
  const r = await fetch(path + sep, opts);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.detail || String(r.status));
  return j;
}

const mapCanvas = document.getElementById("mapCanvas");
if (mapCanvas) {
  const q = id => document.getElementById(id);
  const ctx = mapCanvas.getContext("2d");
  const rulerTop = 30, rulerLeft = 52;
  let cells = new Set(), anchors = new Map(), selectedAnchorId = null, tool = "draw", drag = false;
  let start = null, goal = null, route = [], cellPx = 28, viewRow = -10, viewColumn = -15, panStart = null;
  let measureStart = null, measureEnd = null, liveState = null, mapsLoaded = false;
  const key = (r, c) => r + "," + c, parseKey = k => k.split(",").map(Number), value = id => Number(q(id).value);

  // Messages show under the canvas and under the anchor buttons, since the
  // two are a screen apart.
  function status(message, error = false) {
    for (const id of ["mapStatus", "mapAnchorStatus"]) {
      q(id).textContent = message; q(id).className = error ? "map-status error" : "map-status";
    }
  }
  function centreOn(cell) {
    viewColumn = cell[1] + .5 - (mapCanvas.width - rulerLeft) / cellPx / 2;
    viewRow = cell[0] + .5 - (mapCanvas.height - rulerTop) / cellPx / 2;
  }
  function bounds() {
    if (!cells.size) return null;
    const pts = [...cells].map(parseKey), rs = pts.map(x => x[0]), cs = pts.map(x => x[1]);
    return {minR: Math.min(...rs), maxR: Math.max(...rs), minC: Math.min(...cs), maxC: Math.max(...cs)};
  }
  function updateExtent() {
    const b = bounds(), tile = value("mapTileSize");
    if (!b || !Number.isFinite(tile) || tile <= 0) { q("mapExtent").textContent = "Map edges are defined by the outermost drawn path cells."; return; }
    const width = (b.maxC - b.minC + 1) * tile, length = (b.maxR - b.minR + 1) * tile;
    q("mapExtent").textContent = `Map extent: ${width.toFixed(2)} m wide × ${length.toFixed(2)} m long · rows ${b.minR}…${b.maxR}, columns ${b.minC}…${b.maxC} · ${cells.size} cells`;
  }
  const mapX = c => rulerLeft + (c - viewColumn) * cellPx;
  const mapY = r => rulerTop + (r - viewRow) * cellPx;

  function fitCanvasToBox() {
    // Draw at the displayed size so rulers and labels stay readable.
    const w = Math.round(mapCanvas.clientWidth);
    if (!w) return;
    const h = Math.max(360, Math.round(w * 0.76));
    if (mapCanvas.width !== w || mapCanvas.height !== h) { mapCanvas.width = w; mapCanvas.height = h; }
  }

  function draw() {
    fitCanvasToBox();
    const W = mapCanvas.width, H = mapCanvas.height, plotW = W - rulerLeft, plotH = H - rulerTop;
    ctx.fillStyle = "#d9e2e8"; ctx.fillRect(rulerLeft, rulerTop, plotW, plotH);
    const firstC = Math.floor(viewColumn), lastC = Math.ceil(viewColumn + plotW / cellPx);
    const firstR = Math.floor(viewRow), lastR = Math.ceil(viewRow + plotH / cellPx);
    ctx.save(); ctx.beginPath(); ctx.rect(rulerLeft, rulerTop, plotW, plotH); ctx.clip();
    ctx.fillStyle = "#3fa36c";
    for (const k of cells) {
      const [r, c] = parseKey(k), x = mapX(c), y = mapY(r);
      if (x + cellPx < rulerLeft || y + cellPx < rulerTop || x > W || y > H) continue;
      ctx.fillRect(x, y, cellPx, cellPx);
    }
    ctx.fillStyle = "#2f7de1";
    for (const [r, c] of route) ctx.fillRect(mapX(c), mapY(r), cellPx, cellPx);
    for (const [cell, color] of [[start, "#f0b429"], [goal, "#e65a50"]]) {
      if (cell) { ctx.fillStyle = color; ctx.fillRect(mapX(cell[1]), mapY(cell[0]), cellPx, cellPx); }
    }
    // live car trail + position
    const live = liveState && liveState.configured ? liveState : null;
    if (live) {
      ctx.fillStyle = "rgba(255,138,0,0.8)";
      const inset = cellPx * 0.3;
      for (const [r, c] of live.trail || []) ctx.fillRect(mapX(c) + inset, mapY(r) + inset, cellPx - 2 * inset, cellPx - 2 * inset);
    }
    for (const a of anchors.values()) {
      const x = mapX(a.cell[1] + .5), y = mapY(a.cell[0] + .5);
      ctx.fillStyle = a.id === selectedAnchorId ? "#ff54ce" : "#8f42d8";
      ctx.beginPath(); ctx.arc(x, y, Math.max(5, cellPx * .24), 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#351044"; ctx.font = `bold ${Math.max(10, cellPx * .32)}px system-ui`; ctx.textAlign = "left"; ctx.textBaseline = "alphabetic";
      ctx.fillText(a.label || a.id, x + cellPx * .28, y - cellPx * .28);
    }
    if (live && live.position) {
      const p = live.position, x = mapX(p.column + .5), y = mapY(p.row + .5);
      ctx.fillStyle = p.status === "OK" ? "#16a34a" : "#dc2626"; ctx.strokeStyle = "#111"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, Math.max(6, cellPx * .45), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
    ctx.strokeStyle = "rgba(30,45,55,.32)"; ctx.lineWidth = 1; ctx.beginPath();
    for (let c = firstC; c <= lastC; c++) { const x = mapX(c); ctx.moveTo(x, rulerTop); ctx.lineTo(x, H); }
    for (let r = firstR; r <= lastR; r++) { const y = mapY(r); ctx.moveTo(rulerLeft, y); ctx.lineTo(W, y); }
    ctx.stroke();
    ctx.strokeStyle = "rgba(190,80,80,.65)"; ctx.beginPath();
    const x0 = mapX(0), y0 = mapY(0);
    ctx.moveTo(x0, rulerTop); ctx.lineTo(x0, H); ctx.moveTo(rulerLeft, y0); ctx.lineTo(W, y0); ctx.stroke();
    if (measureStart) {
      const pts = [measureStart, measureEnd || measureStart];
      ctx.strokeStyle = "#ff8a00"; ctx.fillStyle = "#ff8a00"; ctx.lineWidth = 3; ctx.setLineDash([7, 5]);
      ctx.beginPath(); ctx.moveTo(mapX(pts[0][1] + .5), mapY(pts[0][0] + .5)); ctx.lineTo(mapX(pts[1][1] + .5), mapY(pts[1][0] + .5)); ctx.stroke();
      ctx.setLineDash([]);
      for (const p of pts) { ctx.beginPath(); ctx.arc(mapX(p[1] + .5), mapY(p[0] + .5), Math.max(5, cellPx * .18), 0, Math.PI * 2); ctx.fill(); }
    }
    ctx.restore();
    ctx.fillStyle = "#243540"; ctx.fillRect(0, 0, W, rulerTop); ctx.fillRect(0, 0, rulerLeft, H);
    ctx.strokeStyle = "#748b9a"; ctx.lineWidth = 1; ctx.beginPath();
    ctx.moveTo(rulerLeft, 0); ctx.lineTo(rulerLeft, H); ctx.moveTo(0, rulerTop); ctx.lineTo(W, rulerTop); ctx.stroke();
    ctx.fillStyle = "#d8e7ef"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.font = `${Math.max(9, Math.min(12, cellPx * .36))}px system-ui`;
    const every = cellPx < 16 ? Math.ceil(16 / cellPx) : 1;
    for (let c = firstC; c <= lastC; c++) if (c % every === 0) ctx.fillText(String(c), mapX(c + .5), rulerTop / 2);
    ctx.textAlign = "right";
    for (let r = firstR; r <= lastR; r++) if (r % every === 0) ctx.fillText(String(r), rulerLeft - 7, mapY(r + .5));
    ctx.textAlign = "center"; ctx.fillStyle = "#f4cb73"; ctx.font = "bold 10px system-ui";
    ctx.fillText("COL", rulerLeft / 2, rulerTop / 2 - 6); ctx.fillText("ROW", rulerLeft / 2, rulerTop / 2 + 6);
    updateExtent();
  }

  function point(ev) { const box = mapCanvas.getBoundingClientRect(); return {x: (ev.clientX - box.left) * mapCanvas.width / box.width, y: (ev.clientY - box.top) * mapCanvas.height / box.height}; }
  function at(ev) { const p = point(ev); if (p.x < rulerLeft || p.y < rulerTop) return null; return [Math.floor(viewRow + (p.y - rulerTop) / cellPx), Math.floor(viewColumn + (p.x - rulerLeft) / cellPx)]; }
  function updateMeasurement() {
    if (!measureStart) { q("mapMeasurement").textContent = "Measurement: select the Measure tool, then click two tiles."; return; }
    if (!measureEnd) { q("mapMeasurement").textContent = `Measurement start: row ${measureStart[0]}, column ${measureStart[1]}. Click the second tile.`; return; }
    const dr = Math.abs(measureEnd[0] - measureStart[0]), dc = Math.abs(measureEnd[1] - measureStart[1]), moves = dr + dc, straight = Math.hypot(dr, dc), tile = value("mapTileSize");
    const metres = Number.isFinite(tile) && tile > 0 ? ` / ${(moves * tile).toFixed(2)} m along grid · straight-line: ${straight.toFixed(2)} tiles / ${(straight * tile).toFixed(2)} m` : "";
    q("mapMeasurement").textContent = `Rows: ${dr} · columns: ${dc} · grid distance: ${moves} tile moves (${moves + 1} tiles including both ends)${metres}`;
  }
  function apply(ev) {
    if (tool === "pan") {
      const p = point(ev);
      if (!panStart) panStart = {x: p.x, y: p.y, row: viewRow, column: viewColumn};
      viewColumn = panStart.column - (p.x - panStart.x) / cellPx; viewRow = panStart.row - (p.y - panStart.y) / cellPx;
      draw(); return;
    }
    const cell = at(ev); if (!cell) { drag = false; return; }
    const k = key(...cell);
    if (tool === "draw") cells.add(k);
    else if (tool === "erase") {
      cells.delete(k);
      for (const [id, a] of anchors) if (key(...a.cell) === k) anchors.delete(id);
      refreshAnchors();
      if (start && key(...start) === k) start = null;
      if (goal && key(...goal) === k) goal = null;
    } else if (tool === "measure") {
      if (!measureStart || measureEnd) { measureStart = cell; measureEnd = null; } else measureEnd = cell;
      updateMeasurement(); drag = false;
    } else if (tool === "anchor") {
      if (!cells.has(k)) { status("An anchor must be placed on a runnable path cell.", true); drag = false; return; }
      q("mapAnchorRow").value = cell[0]; q("mapAnchorColumn").value = cell[1]; drag = false;
      status(`Anchor position (${cell[0]},${cell[1]}): enter an Anchor ID, then Create / update anchor.`);
    } else if (tool === "start") { cells.add(k); start = cell; drag = false; }
    else if (tool === "goal") { cells.add(k); goal = cell; drag = false; }
    if (tool !== "measure") route = [];
    draw();
  }
  mapCanvas.onpointerdown = e => { drag = true; panStart = null; mapCanvas.setPointerCapture(e.pointerId); apply(e); };
  mapCanvas.onpointermove = e => { if (drag && (tool === "draw" || tool === "erase" || tool === "pan")) apply(e); };
  mapCanvas.onpointerup = () => { drag = false; panStart = null; };
  mapCanvas.onpointercancel = mapCanvas.onpointerup;
  document.querySelectorAll("[data-map-tool]").forEach(b => b.onclick = () => {
    tool = b.dataset.mapTool;
    document.querySelectorAll("[data-map-tool]").forEach(x => x.classList.toggle("active", x === b));
  });
  q("mapZoomIn").onclick = () => { cellPx = Math.min(80, cellPx * 1.25); draw(); };
  q("mapZoomOut").onclick = () => { cellPx = Math.max(4, cellPx / 1.25); draw(); };
  q("mapFit").onclick = () => fit();
  q("mapClearMeasure").onclick = () => { measureStart = measureEnd = null; updateMeasurement(); draw(); };
  q("mapClearDraft").onclick = () => {
    if (cells.size && !confirm("Clear the whole draft map (not the saved file)?")) return;
    cells.clear(); anchors.clear(); selectedAnchorId = null; refreshAnchors();
    start = null; goal = null; route = []; measureStart = measureEnd = null; updateMeasurement(); draw();
  };
  q("mapTileSize").oninput = () => { updateExtent(); updateMeasurement(); };
  q("mapFollow").onclick = () => {
    const p = liveState && liveState.position;
    if (!p) { status("Xe chưa có vị trí trên map.", true); return; }
    viewColumn = p.column + .5 - (mapCanvas.width - rulerLeft) / cellPx / 2;
    viewRow = p.row + .5 - (mapCanvas.height - rulerTop) / cellPx / 2;
    draw();
  };

  window.addEventListener("resize", () => draw());

  function fit() {
    fitCanvasToBox();
    const b = bounds();
    if (!b) { viewRow = -10; viewColumn = -15; cellPx = 28; draw(); return; }
    const rows = b.maxR - b.minR + 1, columns = b.maxC - b.minC + 1;
    cellPx = Math.max(4, Math.min(60, (mapCanvas.width - rulerLeft) / (columns + 4), (mapCanvas.height - rulerTop) / (rows + 4)));
    viewRow = b.minR - 2; viewColumn = b.minC - 2; draw();
  }
  function payload() {
    return {
      schema_version: 2, map_id: q("mapId").value, tile_size_m: value("mapTileSize"),
      robot: {width_m: value("mapRobotWidth"), length_m: value("mapRobotLength"), safety_margin_m: value("mapMargin")},
      runnable_cells: [...cells].map(parseKey), visual_anchors: [...anchors.values()],
    };
  }
  function refreshAnchors() {
    const select = q("mapAnchorSelect"), current = selectedAnchorId || "";
    select.replaceChildren(new Option("New anchor", ""));
    for (const a of [...anchors.values()].sort((x, y) => x.id.localeCompare(y.id))) select.add(new Option(a.label ? `${a.id} · ${a.label}` : a.id, a.id));
    select.value = anchors.has(current) ? current : "";
    if (!select.value) selectedAnchorId = null;
    draw();
  }
  q("mapAnchorSelect").onchange = () => {
    selectedAnchorId = q("mapAnchorSelect").value || null;
    const a = selectedAnchorId ? anchors.get(selectedAnchorId) : null;
    q("mapAnchorId").value = a ? a.id : ""; q("mapAnchorLabel").value = a ? (a.label || "") : "";
    q("mapAnchorRow").value = a ? a.cell[0] : ""; q("mapAnchorColumn").value = a ? a.cell[1] : "";
    if (a) centreOn(a.cell);
    draw();
  };
  q("mapSaveAnchor").onclick = () => {
    const id = q("mapAnchorId").value.trim(), label = q("mapAnchorLabel").value.trim();
    const rowText = q("mapAnchorRow").value, colText = q("mapAnchorColumn").value;
    const row = Number(rowText), column = Number(colText);
    if (!id || rowText === "" || colText === "" || !Number.isInteger(row) || !Number.isInteger(column)) { status("Enter an anchor ID and place it on the map.", true); return; }
    if (!cells.has(key(row, column))) { status("Anchor position must be a runnable path cell.", true); return; }
    if (anchors.has(id) && id !== selectedAnchorId) { status("Anchor IDs must be unique.", true); return; }
    if (selectedAnchorId && selectedAnchorId !== id) anchors.delete(selectedAnchorId);
    anchors.set(id, {id, label: label || null, cell: [row, column]});
    selectedAnchorId = id; centreOn([row, column]); refreshAnchors();
    status(`Anchor ${id} at (${row},${column}) added to the map (pink). Press Validate and save to keep it.`);
    mapCanvas.scrollIntoView({behavior: "smooth", block: "center"});
  };
  q("mapDeleteAnchor").onclick = () => {
    if (!selectedAnchorId) { status("Select an anchor to delete.", true); return; }
    anchors.delete(selectedAnchorId); selectedAnchorId = null;
    q("mapAnchorId").value = q("mapAnchorLabel").value = q("mapAnchorRow").value = q("mapAnchorColumn").value = "";
    refreshAnchors(); status("Anchor deleted from the draft map.");
  };
  function selectedAnchor() {
    const a = selectedAnchorId ? anchors.get(selectedAnchorId) : null;
    if (!a) status("Select an anchor first.", true);
    return a;
  }
  q("mapAnchorStart").onclick = () => { const a = selectedAnchor(); if (a) { start = a.cell.slice(); route = []; draw(); } };
  q("mapAnchorGoal").onclick = () => { const a = selectedAnchor(); if (a) { goal = a.cell.slice(); route = []; draw(); } };
  q("mapAnchorHere").onclick = async () => {
    const a = selectedAnchor(); if (!a) return;
    try { liveState = await apiJson("/api/map/anchor", {ref: a.cell}); renderLive(); status(`Vị trí xe đặt lại tại ${a.id}.`); }
    catch (e) { status("lỗi: " + e.message, true); }
  };

  q("mapPlan").onclick = async () => {
    if (!start || !goal) { status("Set a runnable start and goal first.", true); return; }
    try {
      const body = await apiJson("/api/map/plan", {map: payload(), start, goal});
      route = body.cells; draw();
      status(`Route: ${body.distance_m.toFixed(2)} m, ${body.cells.length - 1} tiles, ${body.turn_count} turns.`);
    } catch (e) { route = []; draw(); status(e.message, true); }
  };
  q("mapSave").onclick = async () => {
    try {
      const body = await apiJson("/api/map/save", payload());
      status(`Saved ${body.file}`); await refreshSaved(body.saved);
    } catch (e) { status(e.message, true); }
  };
  q("mapDelete").onclick = async () => {
    const id = q("mapSaved").value || q("mapId").value;
    if (!id || !confirm(`Delete map ${id}?`)) return;
    try { await apiJson("/api/map/delete", {id}); status(`Deleted map ${id}.`); await refreshSaved(); }
    catch (e) { status(e.message, true); }
  };
  async function refreshSaved(selected) {
    try {
      const body = await apiJson("/api/map/maps");
      const sel = q("mapSaved"), keep = selected || sel.value;
      sel.innerHTML = '<option value="">—</option>' + body.maps.map(m => `<option value="${escapeHtml(m.id)}">${escapeHtml(m.id)}${m.source === "bundled" ? " (repo)" : ""}</option>`).join("");
      if (keep && [...sel.options].some(o => o.value === keep)) sel.value = keep;
      return body.maps;
    } catch (e) { status(e.message, true); return []; }
  }
  async function loadMap(id) {
    try {
      const data = await apiJson("/api/map/load?id=" + encodeURIComponent(id));
      q("mapId").value = data.map_id; q("mapTileSize").value = data.tile_size_m;
      q("mapRobotWidth").value = data.robot.width_m; q("mapRobotLength").value = data.robot.length_m; q("mapMargin").value = data.robot.safety_margin_m;
      cells = new Set(data.runnable_cells.map(x => key(...x)));
      anchors = new Map((data.visual_anchors || []).map(a => [a.id, a]));
      selectedAnchorId = null; refreshAnchors(); start = null; goal = null; route = [];
      measureStart = measureEnd = null; updateMeasurement(); fit(); status(`Map ${data.map_id} loaded.`);
    } catch (e) { status(e.message, true); }
  }
  q("mapLoad").onclick = () => { if (q("mapSaved").value) loadMap(q("mapSaved").value); };

  // --- live car position -------------------------------------------------
  const modeSel = q("mapMode");
  modeSel.onchange = () => { q("mapHeading").style.display = modeSel.value === "heading" ? "" : "none"; };
  q("mapUse").onclick = async () => {
    if (!start) { status("Set start trước (Set start, hoặc chọn anchor rồi → start).", true); return; }
    const body = {map: payload(), start};
    if (modeSel.value === "goal") {
      if (!goal) { status("Set goal trước, hoặc chọn 'đi thẳng theo hướng'.", true); return; }
      body.goal = goal;
    } else body.heading = q("mapHeading").value;
    try {
      liveState = await apiJson("/api/map/start", body);
      if (liveState.setup && liveState.setup.path) route = liveState.setup.path;
      renderLive(); status("Đã lưu vị trí xuất phát. Bắt đầu route sẽ đặt xe ở đây và reset bộ đếm ô.");
    } catch (e) { status("lỗi: " + e.message, true); }
  };
  q("mapLiveClear").onclick = async () => {
    if (!confirm("Xoá vị trí xuất phát đã lưu trên Pi?")) return;
    try { liveState = await apiJson("/api/map/clear", {}); renderLive(); status("Đã xoá vị trí xuất phát."); } catch (e) { status("lỗi: " + e.message, true); }
  };
  function renderLive() {
    const st = liveState, pill = q("mapPill"), detail = q("mapDetail");
    if (!st || !st.configured || !st.position) {
      pill.className = "pill pill-idle"; pill.textContent = "chưa lưu xuất phát"; detail.textContent = "";
    } else {
      const p = st.position, setup = st.setup || {};
      pill.className = "pill " + (p.status === "OK" ? "pill-running" : "pill-error"); pill.textContent = p.status;
      const how = setup.goal ? `route tới (${setup.goal.join(",")}), ${setup.path.length - 1} ô` : `hướng ${setup.heading}`;
      detail.textContent = `ô (${p.row},${p.column}) · ${st.tiles} ô đã đi · xuất phát (${setup.start.join(",")}) · ${how} · map ${setup.map_id || "?"}`;
    }
    draw();
  }

  async function pollMap() {
    if (activeTab() !== "map") return;
    if (!mapsLoaded) {
      mapsLoaded = true;
      const maps = await refreshSaved();
      try { liveState = await apiJson("/api/map/state"); } catch (e) { liveState = null; }
      const want = (liveState && liveState.setup && liveState.setup.map_id) || "campus-path";
      const pick = maps.some(m => m.id === want) ? want : (maps[0] && maps[0].id);
      if (pick) { q("mapSaved").value = pick; await loadMap(pick); }
      else status("Không có map nào trên Pi (maps/tile_maps hoặc TILE_MAP_DIR). Vẽ map mới hoặc deploy lại.", true);
      if (liveState && liveState.setup) {
        start = liveState.setup.start || null; goal = liveState.setup.goal || null; route = liveState.setup.path || [];
      }
      renderLive();
      return;
    }
    try { liveState = await apiJson("/api/map/state"); } catch (e) { return; }
    renderLive();
  }
  draw();
  setInterval(pollMap, 500);
}

// Saved map start next to "bắt đầu route" (route start resets the tile count
// and puts the car back there).
const runMapStart = document.getElementById("runMapStart");
async function pollRunMapStart() {
  if (!runMapStart || activeTab() !== "script") return;
  try {
    const st = await apiJson("/api/map/state");
    const setup = st.setup || {};
    runMapStart.textContent = st.saved_start && setup.start
      ? `Map: xuất phát (${setup.start.join(",")})${setup.goal ? ` → goal (${setup.goal.join(",")})` : ` hướng ${setup.heading}`} · bắt đầu route sẽ reset bộ đếm ô về đây (sửa ở tab Map)`
      : "Map: chưa lưu vị trí xuất phát (tab Map)";
  } catch (e) {
    runMapStart.textContent = "";
  }
}
pollRunMapStart();
setInterval(pollRunMapStart, 3000);

// ---------------------------------------------------------------------------
// §14 Pre-calibrate tab: stationary checklist before a run.
// ---------------------------------------------------------------------------
const precalChecks = document.getElementById("precalChecks");
const precalPill = document.getElementById("precalPill");
const precalDetail = document.getElementById("precalDetail");
const precalCaptureStatus = document.getElementById("precalCaptureStatus");

function renderPrecal(st) {
  if (!st || !st.frames) {
    precalPill.className = "pill pill-idle"; precalPill.textContent = "chưa có frame";
    precalChecks.innerHTML = ""; return;
  }
  precalPill.className = "pill " + (st.ready ? "pill-running" : "pill-error");
  precalPill.textContent = st.ready ? "sẵn sàng chạy" : "chưa sẵn sàng";
  precalDetail.textContent = `${st.frames} frame (trung vị)`;
  precalChecks.innerHTML = st.checks.map(c => `<tr class="${c.ok ? "ok" : "bad"}">
    <td class="mark">${c.ok ? "✓" : "✗"}</td>
    <td>${escapeHtml(c.label)}</td>
    <td>${escapeHtml(c.value)}</td>
    <td class="hint">${escapeHtml(c.hint || "")}</td></tr>`).join("");
}

async function pollPrecal() {
  if (activeTab() !== "precal") return;
  try { renderPrecal(await apiJson("/api/precal")); }
  catch (e) { precalPill.className = "pill pill-error"; precalPill.textContent = "lỗi: " + e.message; }
}

async function precalCapture(what) {
  precalCaptureStatus.textContent = "đang lưu…";
  try {
    const j = await apiJson("/api/precal/capture", {what});
    precalCaptureStatus.textContent = "đã apply: " + Object.entries(j.applied).map(([k, v]) => `${k}=${v}`).join(", ");
    renderPrecal(j.precal);
  } catch (e) { precalCaptureStatus.textContent = "lỗi: " + e.message; }
}

if (precalChecks) {
  document.getElementById("precalCaptureVp").onclick = () => precalCapture(["vp_center_deg"]);
  document.getElementById("precalCaptureRow").onclick = () => precalCapture(["vp_horizon_y"]);
  setInterval(pollPrecal, 500);
}

// Soft gate for "bắt đầu route": warn when the pre-run checklist is not green.
async function precalGateOk() {
  try {
    const st = await apiJson("/api/precal");
    if (st.ready || !st.frames) return true;
    const bad = st.checks.filter(c => !c.ok).map(c => "• " + c.label + ": " + c.value).join("\n");
    return window.confirm("Pre-calibrate chưa đạt:\n" + bad + "\n\nVẫn chạy route?");
  } catch (e) {
    return true;  // endpoint missing (older runtime): do not block
  }
}


// §calib ─ Calibration tab · Calib for tuning: two step-by-step wizards ──
// Each step is ticked from the car's live state; the first unticked step is
// the one to do now and shows how. Apply only stages values; Save in the Tune
// tab writes them into the active profile.
const calibTuning = document.getElementById("calibTuning");
const groundStepsEl = document.getElementById("groundSteps");
const turnStepsEl = document.getElementById("turnSteps");
const CALIB_MIN_TURNS = 3;        // measured turns per side before Apply
const GROUND_MAX_RMS_CM = 3.0;    // fit accepted below this error
let calibPrecal = null, calibGround = null, calibGroundErr = "", calibTurn = null, calibScript = null;
let groundFitResult = null, groundApplied = false, turnApplied = null;

function renderWizard(el, steps) {
  let current = steps.findIndex(st => !st.ok);
  if (current < 0) current = steps.length;
  el.innerHTML = steps.map((st, i) => {
    const cls = i < current ? "done" : (i === current ? "current" : "todo");
    const mark = i < current ? "✓" : (i === current ? "▶" : "○");
    const value = st.value ? ` <span class="muted">· ${escapeHtml(st.value)}</span>` : "";
    return `<li class="${cls}"><span class="mark">${mark}</span>${escapeHtml(st.label)}${value}` +
      `<span class="how">${escapeHtml(st.how || "")}</span></li>`;
  }).join("");
  return current;
}

function precalCheck(key) {
  return ((calibPrecal && calibPrecal.checks) || []).find(c => c.key === key) || null;
}

function tuneIdle() { return !!(tuneState && (tuneState.idle || {}).ok); }

function renderGroundWizard() {
  const vp = precalCheck("vp_x");
  const g = calibGround;
  const seams = g ? g.lines_px.length : 0;
  const nearest = Number((document.getElementById("groundNearest") || {}).value);
  const fit = groundFitResult && groundFitResult.fit;
  const steps = [
    {label: "Xe đứng yên", ok: tuneIdle(), value: tuneIdle() ? "" : ((tuneState || {}).idle || {}).reason,
     how: "Dừng script, base về STOP."},
    {label: "Xe thẳng theo vạch gạch", ok: !!(vp && vp.ok), value: vp ? vp.value : "chưa có frame",
     how: vp && vp.hint ? vp.hint : "Xoay xe cho VP (vạch xanh trên stream) nằm giữa 2 vạch vàng."},
    {label: "Camera thấy ≥ 3 vạch ngang và VP", ok: !!(g && seams >= 3 && g.vp_row != null && g.age_s <= 1.0),
     value: g ? `${seams} vạch, VP ${g.vp_row ?? "-"} px` : (calibGroundErr || "chưa có frame"),
     how: "Lùi xe hoặc dọn vật che để thấy ít nhất 3 vạch gạch ngang phía trước."},
    {label: "Đo thước từ tâm trục trước tới vạch tím", ok: nearest > 0, value: nearest > 0 ? `${nearest} cm` : "",
     how: "Vạch thấp nhất đang được tô tím trên stream. Nhập số cm vào ô bên dưới."},
    {label: `Fit, sai số ≤ ${GROUND_MAX_RMS_CM} cm`, ok: !!(fit && fit.rms_cm <= GROUND_MAX_RMS_CM),
     value: fit ? `k ${fit.k_px_cm}, c ${fit.c_cm} cm, rms ${fit.rms_cm} cm` : "",
     how: fit ? "Sai số lớn: kiểm tra xe có thẳng không, số cm có đúng vạch tím không, rồi Fit lại." : "Bấm Fit."},
    {label: "Apply k, c", ok: groundApplied, how: "Bấm Apply k, c."},
  ];
  const current = renderWizard(groundStepsEl, steps);
  document.getElementById("groundFit").disabled = current < 4;
  document.getElementById("groundApply").disabled = !(current === 5 && !groundApplied);
}

async function groundFitClick() {
  const text = document.getElementById("groundText");
  const nearest = Number(document.getElementById("groundNearest").value);
  text.textContent = "đang fit…";
  groundApplied = false;
  try {
    groundFitResult = await apiJson("/api/ground/fit", {nearest_cm: nearest, apply: false});
    const f = groundFitResult.fit;
    text.textContent = `${groundFitResult.points} vạch → từng vạch lệch ${f.residuals_cm.join(", ")} cm`;
  } catch (e) { groundFitResult = null; text.textContent = "lỗi: " + e.message; }
  renderGroundWizard();
}

async function groundApplyClick() {
  const f = groundFitResult && groundFitResult.fit;
  if (!f) return;
  await tunePost("/api/tune", {values: {ground_k_px_cm: Number(f.k_px_cm.toFixed(1)), ground_c_cm: Number(f.c_cm.toFixed(2))}}, "PUT");
  groundApplied = !!(tuneState && Math.abs(Number(tuneState.values.ground_k_px_cm) - Number(f.k_px_cm.toFixed(1))) < 1e-6);
  document.getElementById("groundText").textContent = groundApplied ? "đã Apply k, c. Bấm Save ở tab Tune để lưu vào profile." : "Apply lỗi, xem trạng thái tab Tune.";
  renderGroundWizard();
}

function turnStageText() {
  const st = calibScript;
  if (!st || !st.running) return "";
  const step = st.step || {};
  const n = `${st.current_step}/${st.total}`;
  if (st.approaching) return `${n} · đang tiến tới vạch rẽ (V+1)`;
  if (step.degrees != null) return `${n} · đang rẽ ${step.action === "turn_right" || step.action === "right" ? "phải" : "trái"}: ${st.step_degrees_done}° / ${step.degrees}°`;
  if (step.action === "settle") return `${n} · settle, đang đo sai số rẽ`;
  if (step.tiles != null) return `${n} · đi thẳng ${st.step_tiles_done}/${step.tiles} ô` + (st.current_step === 1 ? " tới chỗ rẽ" : " sau khi rẽ");
  return `${n} · ${step.action || ""}`;
}

function renderTurnWizard() {
  const t = calibTurn || {active: false, log: [], counts: {left: 0, right: 0}};
  const running = !!(calibScript && calibScript.running);
  const tv = (tuneState || {}).values || {}, td = (tuneState || {}).defaults || {};
  const groundOk = groundApplied || (tv.ground_k_px_cm != null && tv.ground_k_px_cm !== td.ground_k_px_cm);
  const center = precalCheck("center");
  const counts = t.counts || {left: 0, right: 0};
  const enough = counts.left >= CALIB_MIN_TURNS && counts.right >= CALIB_MIN_TURNS;
  const steps = [
    {label: "Fit ground k, c xong (bước A)", ok: groundOk, how: "Làm wizard A trước: lượt rẽ cần k, c để biết khoảng cách tới vạch."},
    {label: "Pre-calibrate đạt", ok: !!(calibPrecal && calibPrecal.ready),
     how: "Mở mục Pre-calibrate phía trên và sửa các dòng ✗."},
    {label: "Bắt đầu phiên đo", ok: !!t.active, how: "Bấm Bắt đầu phiên đo (xoá kết quả phiên cũ)."},
    {label: "Đặt xe giữa làn, cách chỗ rẽ ≥ 2 ô", ok: running || (tuneIdle() && !!(center && center.ok)),
     value: center ? center.value : "", how: center && center.hint ? center.hint : "Xe đứng yên, ở giữa 2 vạch dọc, phía trước còn ít nhất 2 ô thẳng rồi mới tới chỗ rẽ."},
    {label: `Chạy thử đủ ${CALIB_MIN_TURNS} lần mỗi bên`, ok: enough,
     value: `trái ${counts.left}/${CALIB_MIN_TURNS}, phải ${counts.right}/${CALIB_MIN_TURNS}`,
     how: running ? "Đang chạy. Theo dõi xe, bấm Dừng nếu xe đi sai." :
       "Bấm Chạy thử rẽ trái/phải: đi thẳng 2 ô → rẽ 90° ở vạch V+1 → settle → đi thẳng 1 ô. Sau mỗi lần, đưa xe về chỗ bắt đầu. Lần nào xe đi sai thì bấm Xoá ở dòng đó."},
    {label: "Apply suggestion", ok: !!turnApplied, how: "Bấm Apply suggestion, rồi Save ở tab Tune để lưu vào profile."},
  ];
  const current = renderWizard(turnStepsEl, steps);
  const canRun = t.active && !running && current >= 3;
  document.getElementById("turnRunLeft").disabled = !canRun;
  document.getElementById("turnRunRight").disabled = !canRun;
  document.getElementById("turnRunStop").hidden = !running;
  document.getElementById("turnCalibApply").disabled = running || !t.suggest || current < 5;
  document.getElementById("turnStage").textContent = turnStageText() || (t.measuring ? "đang đo sai số…" : "");

  const table = document.getElementById("turnLogTable");
  table.hidden = !(t.log || []).length;
  document.getElementById("turnLog").innerHTML = (t.log || []).map(e => {
    const good = e.e_psi != null && !e.fallback_time;
    const result = good ? "✓ đo được" : (e.fallback_time ? "✗ rẽ theo thời gian (không có heading)" : (t.measuring ? "đang đo…" : "✗ không đo được"));
    return `<tr class="${good ? "ok" : "bad"}"><td>${e.id}</td><td>${e.side === "left" ? "trái" : "phải"}</td>` +
      `<td>${e.e_psi ?? "-"}</td><td>${e.e_y_cm ?? "-"}</td><td>${e.omega ?? "-"}</td><td>${result}</td>` +
      `<td><button class="rm" data-calib-del="${e.id}" ${running ? "disabled" : ""}>Xoá</button></td></tr>`;
  }).join("");
  const sug = t.suggest;
  document.getElementById("turnText").textContent = turnApplied
    ? "đã Apply: " + Object.entries(turnApplied).map(([k, v]) => `${k}=${v}`).join(", ")
    : (sug ? "đề xuất: " + Object.entries(sug).map(([k, v]) => `${k}=${v}`).join(", ") : "");
}

async function turnCall(path, body) {
  try {
    const j = await apiJson(path, body === undefined ? {} : body);
    return j;
  } catch (e) {
    document.getElementById("turnText").textContent = "lỗi: " + e.message;
    return null;
  }
}

async function startTurnTest(side) {
  try {
    const sep = qp ? "&" + qp.slice(1) : "";
    const r = await fetch(`/api/calib/turn/test_steps?side=${side}${sep}`);
    const j = await r.json();
    if (!r.ok) throw new Error(j.detail || r.status);
    const body = {steps: j.steps, preset_name: `calib-turn-${side}`, description: "calibration turn test", mode: "tiles"};
    const s = await fetch("/route/script" + qp, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    const sj = await s.json().catch(() => ({}));
    if (!s.ok) throw new Error(JSON.stringify(sj.detail || s.status));
    turnApplied = null;
  } catch (e) { document.getElementById("turnText").textContent = "lỗi chạy thử: " + e.message; }
  pollCalibTuning();
}

async function pollCalibTuning() {
  if (activeTab() !== "precal" || !calibTuning || !calibTuning.open) return;
  const [pre, ground, turn, script] = await Promise.allSettled([
    apiJson("/api/precal"), apiJson("/api/ground/sample"), apiJson("/api/calib/turn"),
    apiJson("/route/script/status"),
  ]);
  calibPrecal = pre.status === "fulfilled" ? pre.value : null;
  calibGround = ground.status === "fulfilled" ? ground.value : null;
  calibGroundErr = ground.status === "rejected" ? ground.reason.message : "";
  calibTurn = turn.status === "fulfilled" ? turn.value : null;
  calibScript = script.status === "fulfilled" ? (script.value.status || null) : null;
  renderGroundWizard();
  renderTurnWizard();
}

if (calibTuning) {
  document.getElementById("groundFit").onclick = groundFitClick;
  document.getElementById("groundApply").onclick = groundApplyClick;
  document.getElementById("groundNearest").oninput = () => { groundFitResult = null; groundApplied = false; renderGroundWizard(); };
  document.getElementById("turnCalibStart").onclick = async () => {
    const t = calibTurn || {};
    if ((t.log || []).length && !confirm("Bắt đầu phiên mới sẽ xoá các lượt rẽ đã đo. Tiếp tục?")) return;
    turnApplied = null;
    calibTurn = await turnCall("/api/calib/turn/start") || calibTurn;
    renderTurnWizard();
  };
  document.getElementById("turnRunLeft").onclick = () => startTurnTest("left");
  document.getElementById("turnRunRight").onclick = () => startTurnTest("right");
  document.getElementById("turnRunStop").onclick = async () => {
    await fetch("/route/script/stop" + qp, {method: "POST"}).catch(() => null);
    pollCalibTuning();
  };
  document.getElementById("turnCalibApply").onclick = async () => {
    const j = await turnCall("/api/calib/turn/apply");
    if (!j) return;
    turnApplied = j.applied;
    tuneState = j.tune; tuneDraft = Object.assign({}, j.tune.values || {}); tuneTouched = false; renderTuneFields();
    renderTurnWizard();
  };
  document.getElementById("turnLog").addEventListener("click", async e => {
    const id = e.target && e.target.dataset ? e.target.dataset.calibDel : null;
    if (!id || !confirm(`Xoá lượt rẽ #${id} khỏi phiên đo?`)) return;
    turnApplied = null;
    calibTurn = await turnCall("/api/calib/turn/delete", {id: Number(id)}) || calibTurn;
    renderTurnWizard();
  });
  calibTuning.addEventListener("toggle", pollCalibTuning);
  setInterval(pollCalibTuning, 1000);
}

})();
