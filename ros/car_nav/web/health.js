"use strict";
// Health tab: checks every device from the page. Loaded after index.html's script and uses its globals
// (api, send, connected, ws, showTab, TABS, esc, enc, $). Backend: ui_bridge /api/health/... (health_runner.py).
//
// Rule of the owner: a check nobody confirmed counts as FAILED. Actuator tests (wheels, speed) need this page's
// heartbeat (/ui/health_hb every 200 ms while the Health tab is open and visible); close the page, switch the
// tab, lose the WebSocket -> the wheels stop within 0.5 s and the run ends as FAILED.
(() => {
  const NAMES = { lidar: "LiDAR", depth: "Depth camera", camera: "Colour camera", imu: "IMU (MPU6050)",
                  wheels: "Wheels test (moves the wheels)", speed: "Speed test (drives forward 1 m, spins)" };
  const HINTS = { lidar: "automatic", depth: "automatic, needs the Astra", camera: "you confirm the picture",
                  imu: "automatic, car standing still", wheels: "you confirm each wheel", speed: "car on the floor, 2 m clear ahead" };
  let H = null, open = false, hbTimer = null, pollTimer = null, hbWs = null, runsShown = "", detail = null;
  const el = (id) => document.getElementById(id);

  // ------------------------------------------------------------------ DOM (tab + bar banner + styles)
  const st = document.createElement("style");
  st.textContent = `
    #hbanner { font-size: 13px; padding: 1px 8px; border: 1px solid var(--line); border-radius: 4px; cursor: pointer;
      white-space: nowrap; flex: none; }
    #hbanner.pass { border-color: var(--text); font-weight: 700; }
    #hbanner.fail { background: var(--alert); border-color: var(--alert); color: #fff; font-weight: 700; }
    #tab-health .chk { display: flex; gap: 8px; align-items: flex-start; margin: 4px 0; }
    #tab-health .chk input { margin-top: 4px; }
    #tab-health .tag { display: inline-block; min-width: 54px; text-align: center; padding: 0 6px; border: 1px solid var(--line);
      border-radius: 4px; font-weight: 700; font-size: 12px; }
    #tab-health .tag.pass { border-color: var(--text); }
    #tab-health .tag.fail { background: var(--alert); border-color: var(--alert); color: #fff; }
    #tab-health .tag.run { background: var(--text); color: #fff; border-color: var(--text); }
    #tab-health .ask { border: 3px solid var(--text); border-radius: 4px; padding: 10px; margin: 10px 0; }
    #tab-health .ask img { max-width: 100%; border: 1px solid var(--line); display: block; margin: 8px 0; }
    #tab-health .ask .big button { font-size: 18px; padding: 10px 26px; font-weight: 700; }
    #tab-health table { border-collapse: collapse; width: 100%; font-size: 13px; }
    #tab-health td { border-top: 1px solid var(--line); padding: 2px 4px; vertical-align: top; }
    #tab-health .bad { color: var(--alert); font-weight: 600; }
    #tab-health .warnbox { border: 2px solid var(--alert); color: var(--alert); padding: 6px 8px; border-radius: 4px; margin: 8px 0; }`;
  document.head.appendChild(st);
  const banner = document.createElement("span");
  banner.id = "hbanner"; banner.textContent = "health: –";
  banner.onclick = () => showTab("health");
  const bar = document.getElementById("bar");
  bar.insertBefore(banner, document.getElementById("gstate"));

  const root = el("tab-health");
  root.innerHTML = `
    <h2>Before you start</h2>
    <p class="muted" style="margin:4px 0">Tick what is true. Anything the observer has not confirmed counts as FAILED.
      Keep this tab open and visible while a wheel or speed test runs: it is the deadman. E-STOP stops everything.</p>
    <label class="chk"><input type="checkbox" id="h_estop"><span>I am at the E-STOP (this page's button or the motor power switch).</span></label>
    <label class="chk"><input type="checkbox" id="h_lift"><span>Wheels test: the wheels are OFF THE GROUND.</span></label>
    <label class="chk"><input type="checkbox" id="h_space"><span>Wheels test: the car is on the floor with clear space (0.5 m all around); pulses are 0.15 m/s for 1.5 s.</span></label>
    <label class="chk"><input type="checkbox" id="h_ahead"><span>Speed test: the car is on the floor, 2 m clear ahead and room to spin.</span></label>
    <h2>Checks</h2>
    <div id="h_list"></div>
    <div class="row"><button id="h_start">Start</button><button id="h_abort">Abort</button>
      <span id="h_hb" class="muted"></span></div>
    <div id="h_msg" class="status" hidden></div>
    <div id="h_ask"></div>
    <div id="h_now"></div>
    <h2>Results</h2>
    <div id="h_res" class="muted">No run yet.</div>
    <h2>Earlier runs</h2>
    <div id="h_runs"></div>`;
  el("h_list").innerHTML = Object.keys(NAMES).map((k) => `<label class="chk"><input type="checkbox" data-c="${k}" checked>
    <span>${NAMES[k]} <span class="muted">· ${HINTS[k]}</span></span></label>`).join("");

  // ------------------------------------------------------------------ heartbeat (WebSocket topic /ui/health_hb)
  function beat() {
    if (!open || document.hidden || !connected) return;
    if (hbWs !== ws) {          // a new WebSocket (reconnect): advertise again
      send({ op: "advertise", topic: "/ui/health_hb", type: "std_msgs/msg/String" });
      hbWs = ws;
    }
    send({ op: "publish", topic: "/ui/health_hb", msg: { data: "1" } });
  }
  function setOpen(on) {
    open = on;
    clearInterval(hbTimer); clearInterval(pollTimer);
    if (on) { hbTimer = setInterval(beat, 200); beat(); poll(); pollTimer = setInterval(poll, 600); }
  }

  // ------------------------------------------------------------------ polling + rendering
  async function poll() {
    try { H = await api("GET", "health"); render(); } catch (e) { el("h_hb").textContent = "backend: " + e.message; }
  }
  async function pollBanner() {
    try { const d = await api("GET", "health"); H = d; bannerText(); } catch (e) {}
  }
  const tag = (s) => `<span class="tag ${s === "pass" ? "pass" : s === "fail" ? "fail" : "run"}">${esc(String(s).toUpperCase())}</span>`;
  function bannerText() {
    const last = H && H.summary && H.summary.last;
    const running = H && H.state === "running";
    banner.className = running ? "" : last ? last.overall : "";
    banner.textContent = running ? "health: running…" : last
      ? `last health check: ${String(last.overall).toUpperCase()}${last.complete ? "" : " (partial)"} at ${(last.started || "").slice(5, 16)}${H.summary.require && !H.summary.passed_today ? " · NAV blocked" : ""}`
      : "health: not checked";
  }
  function selected() { return [...document.querySelectorAll("#h_list [data-c]")].filter((c) => c.checked).map((c) => c.dataset.c); }
  function render() {
    bannerText();
    const running = H.state === "running";
    el("h_start").disabled = running;
    el("h_abort").disabled = !running;
    el("h_hb").textContent = H.estop ? "E-STOP is ON" : H.goal_active ? "a navigation goal is active" : "";
    const run = H.run;
    el("h_now").innerHTML = run && running ? `<div class="status"><b>${esc(NAMES[run.current] || "")}</b> · ${esc(run.step || "")}</div>` : "";
    renderAsk(run && running ? run.prompt : null);
    if (run) renderResults(run);
    if (++tick % 8 === 1) renderRuns();
  }
  let tick = 0;
  function renderAsk(p) {
    const box = el("h_ask");
    if (!p) { box.innerHTML = ""; box.dataset.pid = ""; return; }
    const left = Math.max(0, Math.round(p.deadline - Date.now() / 1000));
    if (box.dataset.pid === String(p.id) + p.title) { const c = box.querySelector(".cd"); if (c) c.textContent = left; return; }
    box.dataset.pid = String(p.id) + p.title;
    let h = `<div class="ask"><b>${esc(p.title)}</b><p>${esc(p.text)}</p>`;
    if (p.image) h += `<img src="/api/health/snapshot?t=${Date.now()}" alt="colour camera snapshot">`;
    if (p.kind === "number") h += `<div class="row"><input id="h_tape" type="number" step="0.01" min="0" placeholder="tape distance, m (optional)" style="flex:1"></div>`;
    if (p.kind === "notice") h += `<span class="muted">Watch the car. <span class="cd">${left}</span> s</span>`;
    else h += `<div class="row big"><button data-a="yes">${p.kind === "image" ? "Looks right" : "Yes"}</button>
      <button data-a="no">${p.kind === "image" ? "Does NOT look right" : "No"}</button>
      <span class="muted">no answer in <span class="cd">${left}</span> s = FAILED</span></div>`;
    box.innerHTML = h + "</div>";
    box.querySelectorAll("[data-a]").forEach((b) => b.onclick = async () => {
      const tape = el("h_tape") ? el("h_tape").value : undefined;
      box.querySelectorAll("button").forEach((x) => x.disabled = true);
      try { await api("POST", "health/answer", { prompt_id: p.id, answer: b.dataset.a, value: tape }); } catch (e) { msg(e.message); }
      poll();
    });
  }
  function msg(t, bad) { const m = el("h_msg"); m.hidden = !t; m.textContent = t || ""; m.classList.toggle("error", !!bad); }
  function rules(r) {
    return `<table>${(r.rules || []).map((x) => `<tr><td>${x.ok ? "ok" : "<span class='bad'>FAIL</span>"}</td><td>${esc(x.name)}</td>
      <td class="${x.ok ? "" : "bad"}">${esc(x.value)}</td><td class="muted">${esc(x.rule)}</td></tr>`).join("")}</table>`;
  }
  function renderResults(run) {
    const out = [];
    for (const k of run.checks) {
      const r = run.results[k];
      const st = r ? r.status : run.current === k ? "running" : "waiting";
      let body = "";
      if (r) {
        body += `<div class="${r.status === "pass" ? "muted" : "bad"}">${esc(r.detail || "")}</div>${rules(r)}`;
        if (r.steps) body += `<table>${r.steps.map((s) => `<tr><td>${tag(s.status)}</td><td>${esc(s.name)}</td><td class="muted">${esc(s.detail)}</td></tr>`).join("")}</table>`;
        if (r.hint) body += `<div class="muted">hint: camera z ≈ ${r.hint.camera_z_m} m, pitch ≈ ${r.hint.camera_pitch_deg}° down (${esc(r.hint.note)})</div>`;
        if (r.snapshot_b64) body += `<img src="data:${esc(r.snapshot_mime)};base64,${r.snapshot_b64}" style="max-width:240px;border:1px solid var(--line)">`;
        if (r.units_per_mps_proposed !== undefined) {
          body += `<div class="box"><b>real speed ${r.real_mps} m/s</b> of ${r.commanded_mps} commanded (${(100 * r.ratio).toFixed(0)} %, measured by ${esc(r.source)}${r.tape_m ? ", corrected by the tape" : ""}).
            ${r.spin && r.spin.yaw_efficiency !== undefined ? `<br>Spin: ${r.spin.real_rps} rad/s of ${r.spin.commanded_rps}: <b>yaw_efficiency ≈ ${r.spin.yaw_efficiency}</b> (information, not written anywhere).` : ""}
            <br>ugv.units_per_mps: now ${r.units_per_mps_now} → proposed <b>${r.units_per_mps_proposed}</b>
            ${run.run_id && r.status === "pass" ? `<div class="row"><button data-apply="${esc(run.run_id)}">Apply to car.yaml</button><span class="muted">only while the car is stopped</span></div>` : `<span class="muted"> (only a passed test can be applied)</span>`}</div>`;
        }
      }
      out.push(`<div class="box"><div class="row">${tag(st)}<b>${esc(NAMES[k])}</b></div>${body}</div>`);
    }
    const head = run.overall ? `<div class="status ${run.overall === "fail" ? "error" : ""}"><b>${run.overall.toUpperCase()}</b> · ${esc(run.finished || "")}${run.abort_reason ? " · stopped: " + esc(run.abort_reason) : ""}</div>` : "";
    const key = JSON.stringify([run.results, run.overall, run.current]);
    if (el("h_res").dataset.key === key) return;
    el("h_res").dataset.key = key;
    el("h_res").innerHTML = head + out.join("");
    el("h_res").querySelectorAll("[data-apply]").forEach((b) => b.onclick = async () => {
      try { const d = await api("POST", "health/apply_speed", { id: b.dataset.apply }); b.textContent = "Written"; msg(`car.yaml: ugv.units_per_mps = ${d.units_per_mps} (backup ${d.backup})`); }
      catch (e) { msg("not applied: " + e.message, true); }
    });
  }
  async function renderRuns() {
    const runs = (H.summary && H.summary.last) ? await api("GET", "health/runs").then((d) => d.runs).catch(() => []) : [];
    const key = JSON.stringify(runs.map((r) => r.id + r.overall));
    if (key === runsShown) return;
    runsShown = key;
    el("h_runs").innerHTML = runs.slice(0, 20).map((r) => `<div class="box"><div class="row">${tag(r.overall)}<span>${esc(r.started)}</span>
      <span class="muted">${Object.entries(r.checks).map(([k, v]) => k + " " + v).join(" · ")}</span></div>
      <div class="row"><button data-show="${esc(r.id)}">Show</button><a href="/api/health/runs/${enc(r.id)}?download=1">Download</a>
      <button data-del="${esc(r.id)}">Delete</button></div></div>`).join("") || "<p class='muted'>None yet.</p>";
    el("h_runs").querySelectorAll("[data-show]").forEach((b) => b.onclick = async () => {
      const d = await api("GET", "health/runs/" + enc(b.dataset.show));
      el("h_res").dataset.key = ""; renderResults({ ...d, prompt: null }); el("h_res").dataset.key = "x";
    });
    el("h_runs").querySelectorAll("[data-del]").forEach((b) => b.onclick = async () => {
      if (confirm("Delete this health run?")) { await api("DELETE", "health/runs/" + enc(b.dataset.del)); runsShown = ""; poll(); }
    });
  }

  // ------------------------------------------------------------------ actions
  el("h_start").onclick = async () => {
    msg("");
    el("h_res").dataset.key = "";
    try {
      await api("POST", "health/start", { checks: selected(), confirm: {
        estop_in_reach: el("h_estop").checked, wheels_off_ground: el("h_lift").checked,
        clear_space: el("h_space").checked, clear_ahead: el("h_ahead").checked } });
    } catch (e) { msg(e.message, true); }
    poll();
  };
  el("h_abort").onclick = async () => { try { await api("POST", "health/abort", {}); } catch (e) {} poll(); };

  // ------------------------------------------------------------------ hook into the page's tabs
  TABS.health = { view: false, topics: [] };
  const baseShowTab = showTab;
  showTab = function (t) { baseShowTab(t); setOpen(t === "health"); };
  document.addEventListener("visibilitychange", () => { if (open && !document.hidden) poll(); });
  setInterval(() => { if (!open) pollBanner(); }, 10000);
  pollBanner();
  if (new URLSearchParams(location.search).get("tab") === "health") showTab("health");
})();
