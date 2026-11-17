/* ============================================================
   Mission-Control HUD — UI controller
   ============================================================ */
(function () {
  "use strict";
  const $ = (s, c = document) => c.querySelector(s);
  const $$ = (s, c = document) => [...c.querySelectorAll(s)];
  const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;

  /* ---------- Boot sequence ---------- */
  const BOOT_LINES = [
    "ADV FLIGHT COMPUTER v5.2 — POWER ON SELF TEST",
    "► CPU ................................ OK",
    "► MEMORY ............................. OK",
    "► IMU / NAV SENSORS .................. OK",
    "► GUIDANCE MODULE .................... OK",
    "► CONTROL ACTUATORS .................. OK",
    "► SEQUENCER .......................... OK",
    "► TELEMETRY LINK ..................... OK",
    "",
    "ALL SYSTEMS NOMINAL.",
    "LOADING OPERATOR PROFILE: ANANTHU DEV D V ...",
  ];
  const boot = $("#boot");
  const bootLog = $("#boot-log");
  let bootDone = false;

  function endBoot() {
    if (bootDone) return;
    bootDone = true;
    boot.classList.add("done");
    setTimeout(() => boot.remove(), 700);
  }

  if (REDUCED || sessionStorage.getItem("booted-v2")) {
    endBoot();
  } else {
    let li = 0, ci = 0;
    (function type() {
      if (bootDone) return;
      if (li >= BOOT_LINES.length) {
        try { sessionStorage.setItem("booted-v2", "1"); } catch (e) {}
        setTimeout(endBoot, 500);
        return;
      }
      const line = BOOT_LINES[li];
      if (ci <= line.length) {
        bootLog.textContent = BOOT_LINES.slice(0, li).join("\n") + (li ? "\n" : "") + line.slice(0, ci);
        ci += 3;
        setTimeout(type, 8);
      } else {
        li++; ci = 0;
        setTimeout(type, line === "" ? 60 : 120);
      }
    })();
    addEventListener("keydown", endBoot, { once: true });
    boot.addEventListener("pointerdown", endBoot, { once: true });
  }

  /* ---------- UTC clock ---------- */
  const clockEl = $("#tb-clock");
  setInterval(() => {
    const d = new Date();
    clockEl.textContent = d.toISOString().slice(11, 19) + " UTC";
  }, 1000);

  /* ---------- Footer year ---------- */
  $("#year").textContent = new Date().getFullYear();

  /* ---------- Typed roles ---------- */
  const ROLES = [
    "FLIGHT SOFTWARE DEVELOPMENT",
    "GUIDANCE · NAVIGATION · CONTROL",
    "EMBEDDED SYSTEMS",
    "AEROSPACE SIMULATION",
    "SYSTEMS INTEGRATION",
  ];
  const typedEl = $("#typed-roles");
  if (REDUCED) {
    typedEl.textContent = ROLES[0];
  } else {
    let ri = 0, rc = 0, del = false;
    (function roll() {
      const word = ROLES[ri];
      typedEl.textContent = word.slice(0, rc);
      if (!del && rc < word.length) { rc++; setTimeout(roll, 45); }
      else if (!del) { del = true; setTimeout(roll, 1800); }
      else if (rc > 0) { rc--; setTimeout(roll, 18); }
      else { del = false; ri = (ri + 1) % ROLES.length; setTimeout(roll, 300); }
    })();
  }

  /* ---------- Scroll spy + scene module sync ---------- */
  const links = $$("#navlist a");
  const sections = $$("main > section");
  function activate(id) {
    links.forEach(a => a.classList.toggle("active", a.getAttribute("href") === "#" + id));
    if (window.HUD_SCENE) window.HUD_SCENE.setModule(id);
  }
  const spy = new IntersectionObserver(
    (entries) => {
      for (const e of entries) if (e.isIntersecting) activate(e.target.id);
    },
    { rootMargin: "-45% 0px -45% 0px" }
  );
  sections.forEach(s => spy.observe(s));

  /* ---------- Panel reveal ---------- */
  const reveal = new IntersectionObserver(
    (entries) => entries.forEach(e => e.isIntersecting && e.target.classList.add("revealed")),
    { threshold: 0.15 }
  );
  $$(".panel").forEach(p => reveal.observe(p));

  /* ---------- Sequencer ---------- */
  const PHASES = [
    ["BOOT", "POST complete. Subsystems registered.\nNAV alignment in progress ...\nSTATE: SAFE / INHIBITS ACTIVE"],
    ["ARM", "Arming criteria satisfied.\nInhibits released. Batteries on internal.\nSTATE: ARMED / AWAITING LAUNCH CMD"],
    ["LAUNCH", "Launch command received.\nBooster ignition. Liftoff confirmed.\nSTATE: BOOST / RATE DAMPING ACTIVE"],
    ["MIDCOURSE", "Guidance loop engaged.\nTrajectory correction commands issued.\nSTATE: MIDCOURSE / GNC NOMINAL"],
    ["TERMINAL", "Target acquired by seeker.\nTerminal guidance authority transferred.\nSTATE: TERMINAL / INTERCEPT GEOMETRY LOCKED"],
  ];
  const seqButtons = $$(".seq-state");
  const readout = $("#seq-readout");
  let seqTimer = null;

  function setPhase(i, fromAuto) {
    if (!fromAuto && seqTimer) { clearInterval(seqTimer); seqTimer = null; autoBtn.textContent = "▶ RUN SEQUENCE"; }
    seqButtons.forEach((b, j) => b.classList.toggle("active", j <= i));
    const [name, text] = PHASES[i];
    readout.textContent = `[T${i >= 2 ? "+" : "-"}${String(Math.abs(2 - i) * 10).padStart(3, "0")}s] PHASE: ${name}\n${text}`;
    if (window.HUD_SCENE) window.HUD_SCENE.setPhase(i);
  }
  seqButtons.forEach((b, i) => b.addEventListener("click", () => setPhase(i)));
  setPhase(0);

  const autoBtn = $("#seq-auto");
  autoBtn.addEventListener("click", () => {
    if (seqTimer) { clearInterval(seqTimer); seqTimer = null; autoBtn.textContent = "▶ RUN SEQUENCE"; return; }
    let i = 0;
    setPhase(0, true);
    autoBtn.textContent = "■ STOP";
    seqTimer = setInterval(() => {
      i++;
      if (i >= PHASES.length) { clearInterval(seqTimer); seqTimer = null; autoBtn.textContent = "▶ RUN SEQUENCE"; return; }
      setPhase(i, true);
    }, 1700);
  });

  /* ---------- GNC replay ---------- */
  $("#gnc-replay").addEventListener("click", () => {
    if (window.HUD_SCENE) window.HUD_SCENE.playIntercept();
  });

  /* ---------- Hangar switcher ---------- */
  $$("#hangar-ctl .btn").forEach(b =>
    b.addEventListener("click", () => {
      $$("#hangar-ctl .btn").forEach(x => x.classList.remove("active"));
      b.classList.add("active");
      if (window.HUD_SCENE) window.HUD_SCENE.setHangarModel(b.dataset.model);
    })
  );

  /* ---------- Telemetry strip charts ---------- */
  const charts = $$(".tele-chart");
  function drawCharts(ts) {
    for (const c of charts) {
      const dpr = Math.min(devicePixelRatio, 2);
      const w = c.clientWidth, h = c.clientHeight;
      if (!w) continue;
      if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
      const ctx = c.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = "rgba(255,171,64,0.15)";
      ctx.lineWidth = 1;
      for (let gy = 0; gy <= h; gy += h / 3) {
        ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
      }
      const kind = c.dataset.kind;
      ctx.strokeStyle = "#ffab40";
      ctx.lineWidth = 1.4;
      ctx.shadowColor = "rgba(255,171,64,0.6)";
      ctx.shadowBlur = 4;
      ctx.beginPath();
      const t0 = (REDUCED ? 0 : ts) / 1000;
      for (let x = 0; x <= w; x += 2) {
        const u = x / w * 6 + t0 * (kind === "gyro" ? 2.2 : 1.4);
        let v;
        if (kind === "accel") v = Math.sin(u * 2.1) * 0.5 + Math.sin(u * 5.3) * 0.25 + Math.sin(u * 13.7) * 0.1;
        else if (kind === "gyro") v = Math.sin(u * 1.3) * 0.7 + Math.sin(u * 9.1) * 0.15;
        else v = Math.sin(u * 0.6) * 0.8 + Math.sin(u * 3.1) * 0.12;
        const y = h / 2 - v * h * 0.38;
        x === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.shadowBlur = 0;
    }
    if (!REDUCED) requestAnimationFrame(drawCharts);
  }
  requestAnimationFrame(drawCharts);

  /* ---------- Field notes lightbox ---------- */
  const lightbox = $("#lightbox");
  const lbImg = $("#lightbox-img");
  $("#field-grid").addEventListener("click", (e) => {
    const img = e.target.closest("img");
    if (!img) return;
    lbImg.src = img.src;
    lbImg.alt = img.alt;
    lightbox.hidden = false;
  });
  function closeLb() { lightbox.hidden = true; lbImg.src = ""; }
  $("#lightbox-close").addEventListener("click", closeLb);
  lightbox.addEventListener("click", (e) => { if (e.target === lightbox) closeLb(); });
  addEventListener("keydown", (e) => { if (e.key === "Escape" && !lightbox.hidden) closeLb(); });
})();
