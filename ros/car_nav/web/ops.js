"use strict";
// Operations tabs of the web UI: Manual (drive by hand), Runs (rosbag recordings), Logs (pipeline status + /rosout).
// Loaded after the main script of index.html, which it hooks into (TABS, showTab, send, window.onUiTopic, onUiOpen,
// onUiClose, onSyncSubs, onOpsMode). Everything here is idle unless its tab is open.
(function () {
  const $o = (id) => document.getElementById(id);
  const escH = (t) => String(t ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);

  // ------------------------------------------------------------------ tabs + styles
  const style = document.createElement("style");
  style.textContent = `
    #manualbadge { font-weight: 700; padding: 1px 8px; border: 2px solid var(--text); border-radius: 4px; background: var(--text); color: #fff; flex: none; }
    table.pipe { width: 100%; border-collapse: collapse; font-size: 13px; }
    table.pipe td, table.pipe th { padding: 2px 6px; border-bottom: 1px solid var(--soft); text-align: left; vertical-align: top; }
    table.pipe th { font-size: 12px; text-transform: uppercase; letter-spacing: .04em; color: var(--muted); }
    table.pipe tr.grp td { background: var(--soft); font-weight: 600; }
    table.pipe td.n { text-align: right; white-space: nowrap; font-variant-numeric: tabular-nums; }
    table.pipe tr.absent td { color: var(--muted); }
    table.pipe tr.absent td:first-child { text-decoration: line-through; }
    table.pipe tr.stale td.age { color: var(--warn); font-weight: 600; }
    table.pipe td.last { color: var(--muted); font-size: 12px; word-break: break-word; }
    #logbox { height: 340px; overflow: auto; margin: 6px 0; padding: 6px; border: 1px solid var(--line); border-radius: 4px;
      background: #fafafa; font: 12px/1.35 ui-monospace, Menlo, Consolas, monospace; white-space: pre-wrap; word-break: break-word; }
    #logbox .w { color: var(--warn); } #logbox .e { color: var(--alert); font-weight: 600; } #logbox .d { color: var(--muted); }
    #joy { width: 240px; height: 240px; max-width: 100%; border: 1px solid var(--text); border-radius: 50%; touch-action: none;
      background: #fff; display: block; margin: 8px auto; cursor: grab; }
    #mstop { width: 100%; padding: 12px; font-weight: 700; font-size: 16px; }
    .run a { color: var(--text); }
  `;
  document.head.appendChild(style);

  const NEW_TABS = { manual: ["Manual", { view: true, topics: ["map", "pose", "scan", "depth", "guard", "plan"] }],
                     runs: ["Runs", { view: false, topics: ["runs"] }],
                     logs: ["Logs", { view: false, topics: ["pipe", "logs"] }] };
  const side = $o("side");
  for (const [id, [label, spec]] of Object.entries(NEW_TABS)) {
    TABS[id] = spec;
    const b = document.createElement("button");
    b.dataset.tab = id; b.textContent = label; b.onclick = () => showTab(id);
    $o("tabs").appendChild(b);
  }
  const badge = document.createElement("span");
  badge.id = "manualbadge"; badge.textContent = "MANUAL"; badge.hidden = true;
  $o("bar").insertBefore(badge, $o("gstate"));

  // ================================================================== Manual
  const mt = document.createElement("div");
  mt.className = "tab"; mt.id = "tab-manual";
  mt.innerHTML = `
    <div class="status" id="m_state">–</div>
    <label class="row"><input type="checkbox" id="m_on"> <b>Manual control</b> (the page sends commands while this is on)</label>
    <label class="slider"><span>Speed limit<b><output id="m_vo">0.20</output> m/s</b></span>
      <input type="range" id="m_vmax" min="0.05" max="0.4" step="0.01" value="0.2"></label>
    <label class="slider"><span>Turn-rate limit<b><output id="m_wo">0.60</output> rad/s</b></span>
      <input type="range" id="m_wmax" min="0.2" max="1.5" step="0.05" value="0.6"></label>
    <canvas id="joy" width="240" height="240"></canvas>
    <div class="row" style="justify-content:center"><span class="muted">sending</span> <b id="m_cmd">v 0.00 m/s, w 0.00 rad/s</b></div>
    <button id="mstop">STOP (zero + manual off)</button>
    <div id="m_rev" class="muted" style="margin-top:8px"></div>
    <p class="muted">Drag the stick (touch or mouse) or use the keyboard: W / ↑ forward, A / ← turn left, D / → turn right,
      Space = stop. Forward is up, left turns the car left. The page sends 20 commands a second; if they stop for 0.3 s
      (page closed, Wi-Fi lost, tab hidden, stick released and key up) the car stops. Manual commands go through the
      same collision monitor and safety guard as Nav2's. Taking control cancels the active Nav2 goal; sending a goal ends
      manual control. E-STOP overrides everything. It works only while a map mode runs (Map tab).</p>`;
  side.appendChild(mt);

  const M = { on: false, jx: 0, jy: 0, keys: new Set(), timer: null, t0: 0, state: {}, stateT: 0, last: [0, 0], ptr: null };
  const vmaxLimit = () => (M.state.max_speed_mps ?? (D.static && D.static.cruise_speed_mps) ?? 0.4);
  const wmaxLimit = () => (M.state.max_yaw_rps ?? 1.5);
  const reverseOk = () => !!M.state.allow_reverse;

  function setSliderLimits() {
    const v = $o("m_vmax"), w = $o("m_wmax");
    v.max = Math.max(0.05, vmaxLimit()).toFixed(2); w.max = Math.max(0.2, wmaxLimit()).toFixed(2);
    if (+v.value > +v.max) v.value = v.max;
    if (+w.value > +w.max) w.value = w.max;
    $o("m_vo").textContent = (+v.value).toFixed(2); $o("m_wo").textContent = (+w.value).toFixed(2);
  }
  $o("m_vmax").oninput = $o("m_wmax").oninput = setSliderLimits;

  function command() {
    let fx = M.jx, fy = M.jy;                       // stick: x right, y forward (-1..1)
    const k = M.keys;
    const kf = (k.has("w") || k.has("arrowup") ? 1 : 0) - (k.has("s") || k.has("arrowdown") ? 1 : 0);
    const kt = (k.has("a") || k.has("arrowleft") ? 1 : 0) - (k.has("d") || k.has("arrowright") ? 1 : 0);
    if (kf || kt) { fy = kf; fx = -kt; }
    let v = fy * +$o("m_vmax").value, w = -fx * +$o("m_wmax").value;
    if (v < 0 && !reverseOk()) v = 0;               // the car does not reverse unless drive.allow_reverse
    return [v, w];
  }
  function sendTeleop(v, w) {
    send({ op: "publish", topic: "/cmd_vel_teleop", msg: { linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } } });
  }
  function tickTeleop() {
    if (!M.on || document.hidden || !connected || tab !== "manual") return setManual(false, "tab left");
    const [v, w] = command();
    M.last = [v, w];
    sendTeleop(v, w);
    $o("m_cmd").textContent = `v ${v.toFixed(2)} m/s, w ${w.toFixed(2)} rad/s`;
    drawJoy();
  }
  function setManual(on, why) {
    if (on === M.on) { if (!on && why) $o("m_state").textContent = why; return; }
    M.on = on; $o("m_on").checked = on;
    if (on) {
      M.t0 = performance.now(); M.jx = M.jy = 0; M.keys.clear();
      sendTeleop(0, 0);
      M.timer = setInterval(tickTeleop, 50);
    } else {
      clearInterval(M.timer); M.timer = null; M.jx = M.jy = 0; M.keys.clear();
      for (let i = 0; i < 3; i++) sendTeleop(0, 0);   // zeros first; the mux also stops on its own after 0.3 s
      $o("m_cmd").textContent = "v 0.00 m/s, w 0.00 rad/s";
      drawJoy();
    }
    showManualState(why);
  }
  $o("m_on").onchange = () => {
    if ($o("m_on").checked && !(M.state && Object.keys(M.state).length)) {
      $o("m_on").checked = false; $o("m_state").textContent = "Manual drive needs a running map mode: choose one on the Map tab."; return;
    }
    setManual($o("m_on").checked);
  };
  $o("mstop").onclick = () => { setManual(false, "stopped"); };

  function showManualState(why) {
    const s = M.state || {}, el = $o("m_state");
    const have = Object.keys(s).length > 0 && performance.now() - M.stateT < 4000;
    $o("m_on").disabled = !have && !M.on;
    el.className = "status";
    if (!have) el.textContent = "Not available: teleop_mux is not running. Start a map mode (Map tab): the safety guard and collision monitor are part of it.";
    else if (s.estop) el.textContent = "E-STOP is on: manual control is refused until you release it.";
    else if (M.on) el.textContent = s.manual ? "MANUAL: you are driving (Nav2 is paused, goal cancelled)." : "Starting manual control…";
    else el.textContent = (why ? why + ". " : "") + (s.reason === "goal" ? "A navigation goal took over. " : "") + "Manual control is off.";
    $o("m_rev").textContent = reverseOk() ? "Reverse is allowed (drive.allow_reverse)."
      : "Reverse is disabled (drive.allow_reverse: false): backward input is ignored, the car only drives forward and turns.";
    badge.hidden = !(s.manual && have);
  }
  function onMuxState(d) {
    M.state = d; M.stateT = performance.now();
    if (M.on && performance.now() - M.t0 > 700) {
      if (d.estop) setManual(false, "E-STOP");
      else if (!d.manual && d.reason === "goal") setManual(false, "A goal was sent: manual control ended");
    }
    setSliderLimits(); showManualState();
  }
  // joystick
  const joy = $o("joy"), jg = joy.getContext("2d");
  function drawJoy() {
    const w = joy.width, c = w / 2, R = c - 14;
    jg.setTransform(1, 0, 0, 1, 0, 0); jg.clearRect(0, 0, w, w);
    jg.strokeStyle = "#bbb"; jg.lineWidth = 1;
    jg.beginPath(); jg.moveTo(c, 6); jg.lineTo(c, w - 6); jg.moveTo(6, c); jg.lineTo(w - 6, c); jg.stroke();
    jg.fillStyle = "#666"; jg.font = "12px sans-serif"; jg.textAlign = "center";
    jg.fillText("forward", c, 16); jg.fillText(reverseOk() ? "back" : "back (off)", c, w - 8);
    jg.textAlign = "left"; jg.fillText("left", 10, c - 5); jg.textAlign = "right"; jg.fillText("right", w - 10, c - 5);
    const [v, w2] = M.last;
    const px = c + M.jx * R, py = c - M.jy * R;
    jg.fillStyle = M.on ? "#111" : "#999";
    jg.beginPath(); jg.arc(px, py, 22, 0, 2 * Math.PI); jg.fill();
  }
  function joyFrom(e) {
    const r = joy.getBoundingClientRect(), c = [r.left + r.width / 2, r.top + r.height / 2], R = r.width / 2 - 14 * r.width / 240;
    let x = (e.clientX - c[0]) / R, y = -(e.clientY - c[1]) / R;
    const n = Math.hypot(x, y); if (n > 1) { x /= n; y /= n; }
    M.jx = x; M.jy = y; drawJoy();
  }
  joy.addEventListener("pointerdown", (e) => {
    if (!M.on) { $o("m_state").textContent = "Switch Manual control on first."; return; }
    joy.setPointerCapture(e.pointerId); M.ptr = e.pointerId; joyFrom(e);
  });
  joy.addEventListener("pointermove", (e) => { if (M.ptr === e.pointerId) joyFrom(e); });
  const joyEnd = (e) => { if (M.ptr === e.pointerId) { M.ptr = null; M.jx = M.jy = 0; drawJoy(); } };   // release = zero
  joy.addEventListener("pointerup", joyEnd); joy.addEventListener("pointercancel", joyEnd);
  joy.addEventListener("lostpointercapture", joyEnd);
  const KEYS = ["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"];
  window.addEventListener("keydown", (e) => {
    if (!M.on || tab !== "manual" || /INPUT|SELECT|TEXTAREA/.test((document.activeElement || {}).tagName || "") && document.activeElement.type !== "checkbox" && document.activeElement.type !== "range") return;
    const k = e.key.toLowerCase();
    if (k === " ") { M.keys.clear(); M.jx = M.jy = 0; e.preventDefault(); return; }
    if (KEYS.includes(k)) { M.keys.add(k); e.preventDefault(); }
  });
  window.addEventListener("keyup", (e) => M.keys.delete(e.key.toLowerCase()));
  window.addEventListener("blur", () => { M.keys.clear(); M.jx = M.jy = 0; });       // a key held while the window loses focus
  document.addEventListener("visibilitychange", () => { if (document.hidden && M.on) setManual(false, "page hidden"); });
  drawJoy();

  // ==================================================================== Runs
  const rt = document.createElement("div");
  rt.className = "tab"; rt.id = "tab-runs";
  rt.innerHTML = `
    <p class="muted"><b>Run</b> = the full rosbag2 recording (every message of the important topics: scan, odom, TF, pose, plan,
      commands, wheel echo, guard, depth, IMU, /rosout), to replay or analyse later (<code>ros2 bag play</code>).
      A <b>test record</b> (Tests tab) is a light JSON summary of one test. A test started while a run records is linked to it.</p>
    <div class="row"><input id="r_name" placeholder="run name, e.g. corridor_lap" style="flex:1"></div>
    <input id="r_note" placeholder="note (optional)" style="width:100%">
    <div class="row"><button id="r_start" class="on">● Record run</button><button id="r_stop">Stop + save</button></div>
    <div id="r_status" class="status">–</div>
    <div id="r_msg" class="muted"></div>
    <h2>Saved runs</h2>
    <div id="r_list"></div>`;
  side.appendChild(rt);
  const R = { rec: false, listT: 0 };
  const rmsg = (t) => { $o("r_msg").textContent = t; };
  function showRunStatus(s) {
    if (!s) return;
    R.rec = !!s.recording;
    $o("r_start").disabled = R.rec; $o("r_stop").disabled = !R.rec;
    $o("r_status").textContent = (R.rec ? `● recording "${s.name}" ${Math.round(s.elapsed_s)} s, ${s.size_mb} MB. ` : "Not recording. ")
      + `Free disk ${(s.free_mb / 1000).toFixed(1)} GB (recording stops below ${(s.min_free_mb / 1000).toFixed(1)} GB); storage ${s.storage}.`;
    if (R.rec && performance.now() - R.listT > 10000) loadRuns();
  }
  async function loadRuns() {
    R.listT = performance.now();
    try {
      const d = await api("GET", "runs");
      showRunStatus(d.status);
      $o("r_list").innerHTML = d.runs.map((r) => `<div class="box run" data-id="${escH(r.id)}">
          <div class="row"><b>${escH(r.name)}</b><span class="muted">${escH(r.started)}</span></div>
          <div class="muted">${r.state === "recording" ? "● recording" : escH(r.state)} · ${r.duration_s ?? "–"} s · ${r.size_mb} MB · ${escH(r.storage)}
            ${r.stop_reason ? " · " + escH(r.stop_reason) : ""}${(r.tests || []).length ? " · tests: " + r.tests.map(escH).join(", ") : ""}</div>
          <div class="row"><input class="note" placeholder="note" value="${escH(r.note)}" style="flex:1"></div>
          <div class="row"><button class="rsave">Save note</button>
            ${r.state === "recording" ? "" : `<a href="/api/runs/${enc(r.id)}/download">Download (.zip)</a>`}
            <button class="rdel" ${r.state === "recording" ? "disabled" : ""}>Delete</button></div></div>`).join("")
        || "<p class='muted'>No runs yet.</p>";
      $o("r_list").querySelectorAll(".run").forEach((el) => {
        const id = el.dataset.id;
        el.querySelector(".rsave").onclick = async () => {
          await api("PUT", "runs/" + enc(id), { note: el.querySelector(".note").value }); el.querySelector(".rsave").textContent = "Saved";
        };
        el.querySelector(".rdel").onclick = async () => {
          if (!confirm("Delete this run and its bag? This cannot be undone.")) return;
          try { await api("DELETE", "runs/" + enc(id)); loadRuns(); } catch (e) { rmsg(e.message); }
        };
      });
    } catch (e) { $o("r_list").textContent = "runs: " + e.message; }
  }
  $o("r_start").onclick = async () => {
    try { await api("POST", "runs/start", { name: $o("r_name").value.trim() || "run", note: $o("r_note").value }); rmsg(""); loadRuns(); }
    catch (e) { rmsg(e.message); }
  };
  $o("r_stop").onclick = async () => {
    $o("r_stop").disabled = true; rmsg("saving…");
    try { const r = await api("POST", "runs/stop"); rmsg(`saved ${r.id}`); } catch (e) { rmsg(e.message); }
    loadRuns();
  };

  // ==================================================================== Logs
  const lt = document.createElement("div");
  lt.className = "tab"; lt.id = "tab-logs";
  lt.innerHTML = `
    <h2>Pipeline: messages between the stages</h2>
    <div class="muted" id="p_note">Rate over the last 5 s, age of the last message, last value. Only measured while this tab is open.
      A struck-through name has no publisher (that stage is not running).</div>
    <table class="pipe" id="p_table"><thead><tr><th>topic</th><th>Hz</th><th>age</th><th>last value</th></tr></thead><tbody></tbody></table>
    <h2>Log console (/rosout)</h2>
    <div class="row">
      <select id="l_level"><option value="10">DEBUG+</option><option value="20" selected>INFO+</option><option value="30">WARN+</option><option value="40">ERROR+</option></select>
      <input id="l_node" placeholder="node / text filter" style="flex:1">
      <button id="l_pause">Pause</button><button id="l_clear">Clear</button><button id="l_dl">Download</button>
    </div>
    <div id="logbox"></div>
    <div class="muted" id="l_info"></div>`;
  side.appendChild(lt);
  const L = { lines: [], paused: false, dirty: false };
  const LV = { 10: ["DEBUG", "d"], 20: ["INFO", ""], 30: ["WARN", "w"], 40: ["ERROR", "e"], 50: ["FATAL", "e"] };
  const fmtT = (t) => { if (t > 1e9) { const d = new Date(t * 1000); return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0"); } return t.toFixed(3); };
  function logPass(l) {
    const min = +$o("l_level").value, f = $o("l_node").value.trim().toLowerCase();
    return l[1] >= min && (!f || l[2].toLowerCase().includes(f) || l[3].toLowerCase().includes(f));
  }
  function renderLogs() {
    L.dirty = false;
    const box = $o("logbox"), stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 20;
    const shown = L.lines.filter(logPass).slice(-500);
    box.innerHTML = shown.map((l) => { const [n, c] = LV[l[1]] || [l[1], ""];
      return `<div class="${c}">${fmtT(l[0])} ${n.padEnd(5)} [${escH(l[2])}] ${escH(l[3])}</div>`; }).join("");
    if (stick) box.scrollTop = box.scrollHeight;
    $o("l_info").textContent = `${shown.length} of ${L.lines.length} lines kept (last 500)${L.paused ? ", paused" : ""}`;
  }
  const lazyRender = () => { L.dirty = true; if (!L.paused) renderLogs(); };
  $o("l_level").onchange = $o("l_node").oninput = renderLogs;
  $o("l_pause").onclick = () => { L.paused = !L.paused; $o("l_pause").classList.toggle("on", L.paused); $o("l_pause").textContent = L.paused ? "Resume" : "Pause"; if (!L.paused) renderLogs(); };
  $o("l_clear").onclick = () => { L.lines = []; renderLogs(); };
  $o("l_dl").onclick = () => {
    const txt = L.lines.filter(logPass).map((l) => `${fmtT(l[0])} ${(LV[l[1]] || [l[1]])[0]} [${l[2]}] ${l[3]}`).join("\n");
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([txt + "\n"], { type: "text/plain" }));
    a.download = "car-log-" + new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19) + ".txt";
    a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  };
  function onLogs(d) {
    for (const l of d.lines) L.lines.push(l);
    if (L.lines.length > 500) L.lines.splice(0, L.lines.length - 500);
    lazyRender();
  }
  function onPipe(d) {
    let html = "", g = null;
    for (const r of d.stages) {
      if (r.group !== g) { g = r.group; html += `<tr class="grp"><td colspan="4">${escH(g)}</td></tr>`; }
      const st = r.age_s == null ? (r.pubs ? "silent" : "absent") : r.age_s > 2 ? "stale" : "ok";
      html += `<tr class="${st}"><td>${escH(r.topic)}</td><td class="n">${r.age_s == null ? "–" : r.hz.toFixed(1)}</td>`
        + `<td class="n age">${r.age_s == null ? (r.pubs ? "no msg" : "–") : r.age_s.toFixed(1) + " s"}</td><td class="last">${escH(r.last)}</td></tr>`;
    }
    $o("p_table").tBodies[0].innerHTML = html;
    $o("p_note").textContent = `Map mode: ${d.mode} (${d.state}). Rate over the last 5 s, age of the last message, last value. `
      + `A struck-through name has no publisher (that stage is not running); "no msg" = a publisher exists but nothing arrived.`;
  }

  // ============================================================== hooks
  window.onUiTopic = (topic, d) => {
    if (topic === "/ui/pipe") onPipe(d);
    else if (topic === "/ui/logs") onLogs(d);
    else if (topic === "/ui/runs") showRunStatus(d.status);
    else if (topic === "/teleop_mux/state") onMuxState(d);
  };
  let muxSub = false;
  window.onSyncSubs = () => {
    const want = connected && !document.hidden && tab === "manual";
    if (want && !muxSub) { send({ op: "subscribe", topic: "/teleop_mux/state", type: "std_msgs/msg/String", queue_length: 1 }); muxSub = true; }
    else if (!want && muxSub) { send({ op: "unsubscribe", topic: "/teleop_mux/state" }); muxSub = false; }
  };
  window.onUiOpen = () => {
    muxSub = false;
    send({ op: "advertise", topic: "/cmd_vel_teleop", type: "geometry_msgs/msg/Twist" });
  };
  window.onUiClose = () => { muxSub = false; if (M.on) setManual(false, "connection lost"); };
  window.onOpsMode = (d) => {
    const m = d.manual || {};
    if (!M.state || !Object.keys(M.state).length || performance.now() - M.stateT > 4000) {   // 1 Hz summary when the 5 Hz topic is not open
      if (tab !== "manual") badge.hidden = !m.manual;
    }
    if (tab !== "manual") badge.hidden = !m.manual;
    if (tab === "manual" && Object.keys(m).length && performance.now() - M.stateT > 2500) { M.state = { ...M.state, ...m }; M.stateT = performance.now(); showManualState(); }
  };
  const _showTab = showTab;
  showTab = function (t) {
    if (tab === "manual" && t !== "manual" && M.on) setManual(false, "tab left");
    _showTab(t);
    if (t === "runs") loadRuns();
    if (t === "logs") { renderLogs(); }
    if (t === "manual") { setSliderLimits(); showManualState(); drawJoy(); }
    // the Tests tab: say how a test record and a run differ
    const tn = $o("tab-tests");
    if (t === "tests" && tn && !$o("t_runnote")) {
      const p = document.createElement("p"); p.id = "t_runnote"; p.className = "muted";
      p.innerHTML = "A test record is a light JSON summary (events + 5 Hz trace). For every message of the run use the <b>Runs</b> tab (rosbag); a test started while a run records is linked to it.";
      tn.insertBefore(p, tn.firstChild);
    }
  };
  const want = new URLSearchParams(location.search).get("tab") || (() => { try { return localStorage.getItem("tab"); } catch (e) { return null; } })();
  if (NEW_TABS[want]) showTab(want);
})();
