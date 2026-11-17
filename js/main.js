/* ============================================================
   Mission Film — director: scroll, HUD, sound, panels
   ============================================================ */
import { createWorld } from "./world.js";

const $ = (s, c = document) => c.querySelector(s);
const $$ = (s, c = document) => [...c.querySelectorAll(s)];
const clamp01 = (x) => Math.min(1, Math.max(0, x));
const lerp = (a, b, k) => a + (b - a) * k;
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
const MOBILE = matchMedia("(max-width: 860px)").matches;

$("#year").textContent = new Date().getFullYear();

/* ---------- Sound engine (synthesized, off by default) ---------- */
class SoundEngine {
  constructor() { this.on = false; this.ctx = null; }
  ensure() {
    if (this.ctx) return;
    const C = window.AudioContext || window.webkitAudioContext;
    if (!C) return;
    this.ctx = new C();
    this.master = this.ctx.createGain();
    this.master.gain.value = 0;
    this.master.connect(this.ctx.destination);
    const noiseBuf = this.ctx.createBuffer(1, this.ctx.sampleRate * 2, this.ctx.sampleRate);
    const d = noiseBuf.getChannelData(0);
    let last = 0;
    for (let i = 0; i < d.length; i++) { // brown-ish noise
      const w = Math.random() * 2 - 1;
      last = (last + 0.02 * w) / 1.02;
      d[i] = last * 3.2;
    }
    this.noiseBuf = noiseBuf;
    // rumble
    this.rumble = this.loop(noiseBuf, "lowpass", 85, 0);
    // air hiss / space tone
    this.hiss = this.loop(noiseBuf, "bandpass", 2400, 0);
  }
  loop(buf, type, freq, gain) {
    const src = this.ctx.createBufferSource();
    src.buffer = buf; src.loop = true;
    const f = this.ctx.createBiquadFilter();
    f.type = type; f.frequency.value = freq;
    const g = this.ctx.createGain();
    g.gain.value = gain;
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start();
    return g;
  }
  setAmbient(rumble, hiss, dt) {
    if (!this.ctx) return;
    const k = 1 - Math.pow(0.02, dt);
    this.rumble.gain.value = lerp(this.rumble.gain.value, rumble, k);
    this.hiss.gain.value = lerp(this.hiss.gain.value, hiss, k);
  }
  burst(freq, dur, peak, type = "lowpass") {
    if (!this.ctx || !this.on) return;
    const t0 = this.ctx.currentTime;
    const src = this.ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const f = this.ctx.createBiquadFilter();
    f.type = type;
    f.frequency.setValueAtTime(freq * 5, t0);
    f.frequency.exponentialRampToValueAtTime(freq, t0 + dur);
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(peak, t0);
    g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start(t0); src.stop(t0 + dur + 0.1);
    const o = this.ctx.createOscillator();
    o.type = "sine";
    o.frequency.setValueAtTime(44, t0);
    const og = this.ctx.createGain();
    og.gain.setValueAtTime(peak * 0.8, t0);
    og.gain.exponentialRampToValueAtTime(0.001, t0 + dur * 1.2);
    o.connect(og); og.connect(this.master);
    o.start(t0); o.stop(t0 + dur * 1.2 + 0.1);
  }
  beep(freq = 980, dur = 0.09, vol = 0.1) {
    if (!this.ctx || !this.on) return;
    const t0 = this.ctx.currentTime;
    const o = this.ctx.createOscillator();
    o.type = "square"; o.frequency.value = freq;
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(vol, t0);
    g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    o.connect(g); g.connect(this.master);
    o.start(t0); o.stop(t0 + dur + 0.05);
  }
  toggle() {
    this.ensure();
    if (!this.ctx) return false;
    if (this.ctx.state === "suspended") this.ctx.resume();
    this.on = !this.on;
    this.master.gain.linearRampToValueAtTime(this.on ? 0.55 : 0, this.ctx.currentTime + 0.4);
    return this.on;
  }
}
const snd = new SoundEngine();
const sndBtn = $("#sndBtn");
sndBtn.addEventListener("click", () => {
  const on = snd.toggle();
  sndBtn.setAttribute("aria-pressed", on);
  sndBtn.textContent = on ? "SND ◉" : "SND ○";
});

/* ---------- text scramble ---------- */
const GLYPHS = "█▓▒░<>/\\|=+*#";
function scramble(el) {
  if (REDUCED) return;
  const orig = el.dataset.orig || (el.dataset.orig = el.textContent);
  const n = orig.length;
  const t0 = performance.now(), dur = 620;
  (function tick(now) {
    const k = clamp01((now - t0) / dur);
    const solid = Math.floor(k * n);
    let out = orig.slice(0, solid);
    for (let i = solid; i < n; i++) out += orig[i] === " " ? " " : GLYPHS[(Math.random() * GLYPHS.length) | 0];
    el.textContent = out;
    if (k < 1) requestAnimationFrame(tick);
    else el.textContent = orig;
  })(t0);
}

/* ---------- panel reveal ---------- */
const reveal = new IntersectionObserver(
  (es) => es.forEach((e) => {
    if (e.isIntersecting && !e.target.classList.contains("revealed")) {
      e.target.classList.add("revealed");
      const h = e.target.querySelector("h1, h2");
      if (h) scramble(h);
    }
  }),
  { threshold: 0.2 }
);
$$(".panel").forEach((p) => reveal.observe(p));

/* ---------- lightbox ---------- */
const lightbox = $("#lightbox"), lbImg = $("#lightbox-img");
$("#stills").addEventListener("click", (e) => {
  const img = e.target.closest("img");
  if (!img) return;
  lbImg.src = img.src; lbImg.alt = img.alt;
  lightbox.hidden = false;
});
const closeLb = () => { lightbox.hidden = true; lbImg.src = ""; };
$("#lightbox-close").addEventListener("click", closeLb);
lightbox.addEventListener("click", (e) => { if (e.target === lightbox) closeLb(); });
addEventListener("keydown", (e) => { if (e.key === "Escape" && !lightbox.hidden) closeLb(); });

/* ---------- telemetry charts ---------- */
const charts = $$(".tele-chart");
let chartsLive = false;
new IntersectionObserver((es) => es.forEach((e) => (chartsLive = e.isIntersecting)), { threshold: 0.1 })
  .observe(charts[0] || document.body);
function drawCharts(ts) {
  if (!chartsLive) return;
  for (const c of charts) {
    const dpr = Math.min(devicePixelRatio, 2);
    const w = c.clientWidth, h = c.clientHeight;
    if (!w) continue;
    if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
    const x = c.getContext("2d");
    x.setTransform(dpr, 0, 0, dpr, 0, 0);
    x.clearRect(0, 0, w, h);
    x.strokeStyle = "rgba(255,171,64,0.15)";
    x.lineWidth = 1;
    for (let gy = 0; gy <= h; gy += h / 3) { x.beginPath(); x.moveTo(0, gy); x.lineTo(w, gy); x.stroke(); }
    const kind = c.dataset.kind;
    x.strokeStyle = "#ffab40"; x.lineWidth = 1.4;
    x.shadowColor = "rgba(255,171,64,0.6)"; x.shadowBlur = 4;
    x.beginPath();
    const t0 = (REDUCED ? 0 : ts) / 1000;
    for (let px = 0; px <= w; px += 2) {
      const u = (px / w) * 6 + t0 * (kind === "gyro" ? 2.2 : 1.4);
      let v;
      if (kind === "accel") v = Math.sin(u * 2.1) * 0.5 + Math.sin(u * 5.3) * 0.25 + Math.sin(u * 13.7) * 0.1;
      else if (kind === "gyro") v = Math.sin(u * 1.3) * 0.7 + Math.sin(u * 9.1) * 0.15;
      else v = Math.sin(u * 0.6) * 0.8 + Math.sin(u * 3.1) * 0.12;
      const y = h / 2 - v * h * 0.38;
      px === 0 ? x.moveTo(px, y) : x.lineTo(px, y);
    }
    x.stroke();
    x.shadowBlur = 0;
  }
}

/* ---------- film bootstrap ---------- */
const canvas = $("#film");
const loader = $("#loader");
const acts = $$("section.act");
const railDots = $$("#rail button");
const actLabel = $("#actlabel");
const countEl = $("#count");
const calloutEl = $("#callout");
const reticle = $("#reticle");
const cut = $("#cut");
const interceptPanel = $('[data-act="intercept"] .panel');

const CUTS = new Set(["launch>gnc", "gnc>launch", "intercept>orbit", "orbit>intercept", "orbit>station", "station>orbit", "station>hangar", "hangar>station", "hangar>dawn", "dawn>hangar"]);
const LETTERBOX = new Set(["launch", "gnc", "intercept"]);
const ACT_NAMES = {
  title: "ACT 0 · PAD 7 — NIGHT", countdown: "ACT I · OPERATOR", launch: "ACT II · ASCENT",
  gnc: "ACT III · GUIDANCE", intercept: "ACT IV · INTERCEPT", orbit: "ACT V · ORBIT",
  station: "ACT VI · GROUND STATION", hangar: "ACT VII · ARSENAL", dawn: "ACT VIII · DAWN",
};
const AMBIENT = { // [rumble, hiss]
  title: [0.1, 0.012], countdown: [0.12, 0.012], launch: [0.5, 0.02], gnc: [0.1, 0.045],
  intercept: [0.12, 0.045], orbit: [0.015, 0.05], station: [0.07, 0.02], hangar: [0.05, 0.01], dawn: [0.08, 0.015],
};
const CALLOUTS = [
  { act: "launch", q: 0.085, text: "LIFTOFF", snd: () => snd.burst(70, 2.2, 0.8) },
  { act: "launch", q: 0.4, text: "BOOST PHASE", snd: () => snd.beep(760) },
  { act: "launch", q: 0.75, text: "MIDCOURSE", snd: () => snd.beep(880) },
  { act: "gnc", q: 0.22, text: "GUIDANCE ACTIVE", snd: () => snd.beep(980) },
  { act: "gnc", q: 0.65, text: "TARGET TRACK", snd: () => snd.beep(1100, 0.07) },
  { act: "intercept", q: 0.32, text: "TERMINAL LOCK", snd: () => { snd.beep(1240, 0.07); snd.beep(1240, 0.07); } },
  { act: "intercept", q: 0.705, text: "INTERCEPT CONFIRMED", snd: () => snd.burst(110, 1.6, 0.9, "bandpass") },
  { act: "orbit", q: 0.2, text: "PAYLOAD DEPLOYED", snd: () => snd.beep(660, 0.12) },
  { act: "dawn", q: 0.1, text: "MISSION COMPLETE", snd: () => snd.beep(520, 0.14, 0.08) },
];
const armed = CALLOUTS.map(() => true);
let calloutTimer = 0;

let ranges = [];
function measure() {
  const span = document.documentElement.scrollHeight - innerHeight;
  ranges = acts.map((el) => ({
    id: el.dataset.act,
    start: el.offsetTop / span,
    end: (el.offsetTop + el.offsetHeight) / span,
  }));
  if (world) world.setActRanges(ranges);
}

let world = null;
let prevAct = "title";
let pS = 0;

function hud(p, dt) {
  $("#progress-fill").style.transform = `scaleX(${p})`;
  let act = ranges[0], q = 0;
  for (let i = ranges.length - 1; i >= 0; i--) if (p >= ranges[i].start) { act = ranges[i]; break; }
  q = clamp01((p - act.start) / Math.max(act.end - act.start, 1e-4));
  const id = act.id;

  if (id !== prevAct) {
    if (CUTS.has(`${prevAct}>${id}`)) {
      cut.classList.add("dip");
      setTimeout(() => cut.classList.remove("dip"), 420);
    }
    prevAct = id;
    actLabel.textContent = ACT_NAMES[id] || id.toUpperCase();
    railDots.forEach((d) => d.classList.toggle("active", d.dataset.act === id));
    document.body.classList.toggle("letterbox", LETTERBOX.has(id));
  }

  // countdown digits
  if (id === "countdown" && q > 0.25) {
    const n = Math.max(1, Math.ceil(10 * (1 - (q - 0.25) / 0.75)));
    if (countEl.textContent !== String(n)) {
      countEl.textContent = n;
      countEl.classList.remove("tick");
      void countEl.offsetWidth;
      countEl.classList.add("tick");
      snd.beep(n <= 3 ? 1180 : 860, 0.06, 0.07);
    }
    countEl.hidden = false;
  } else if (id === "launch" && q < 0.055) {
    countEl.textContent = "IGNITION";
    countEl.hidden = false;
  } else countEl.hidden = true;

  // callouts
  CALLOUTS.forEach((c, i) => {
    const inAct = c.act === id;
    const past = inAct && q >= c.q;
    if (past && armed[i]) {
      armed[i] = false;
      calloutEl.textContent = c.text;
      calloutEl.classList.add("show");
      calloutTimer = 1.9;
      c.snd && c.snd();
    }
    if (inAct && q < c.q - 0.04) armed[i] = true;
    if (!inAct && c.act !== prevAct) armed[i] = true;
  });
  if (calloutTimer > 0) {
    calloutTimer -= dt;
    if (calloutTimer <= 0) calloutEl.classList.remove("show");
  }

  // seeker reticle — tracks the target on screen
  const ts = world && world.targetScreen;
  if (id === "intercept" && q > 0.16 && q < 0.7 && ts && ts.on) {
    reticle.hidden = false;
    const k = clamp01((q - 0.16) / 0.3);
    reticle.style.left = ts.x + "px";
    reticle.style.top = ts.y + "px";
    reticle.style.transform = `translate(-50%,-50%) scale(${lerp(2.2, 1, k)})`;
    reticle.classList.toggle("locked", q > 0.44);
  } else reticle.hidden = true;

  // late V&V panel after the impact
  if (interceptPanel) interceptPanel.classList.toggle("show-late", id === "intercept" && q > 0.76);

  // ambient sound targets
  const amb = AMBIENT[id] || [0.08, 0.01];
  snd.setAmbient(snd.on ? amb[0] : 0, snd.on ? amb[1] : 0, dt);
}

(async () => {
  world = await createWorld(canvas, { tier: MOBILE ? "low" : "hi" });
  if (!world) {
    document.body.classList.add("no-webgl");
    loader.classList.add("done");
    setTimeout(() => loader.remove(), 800);
    return;
  }
  measure();
  addEventListener("resize", () => { measure(); world.resize(); });

  // pointer parallax + hangar drag
  let dragging = false, px0 = 0;
  addEventListener("pointermove", (e) => {
    world.pointer.x = (e.clientX / innerWidth) * 2 - 1;
    world.pointer.y = (e.clientY / innerHeight) * 2 - 1;
    if (dragging) {
      world.hangarYaw += (e.clientX - px0) * 0.006;
      px0 = e.clientX;
    }
  });
  addEventListener("pointerdown", (e) => {
    if (world.currentAct !== "hangar" || e.target.closest(".panel, a, button")) return;
    dragging = true; px0 = e.clientX;
  });
  addEventListener("pointerup", () => (dragging = false));

  loader.classList.add("done");
  setTimeout(() => loader.remove(), 900);
  $("#hint").classList.add("show");
  let hintHidden = false;

  if (REDUCED) {
    world.update(0.005, 0.016, 0.4);
    document.body.classList.add("reduced");
    return;
  }

  let last = performance.now();
  (function frame(now) {
    const dt = Math.min((now - last) / 1000, 0.05);
    last = now;
    const span = document.documentElement.scrollHeight - innerHeight;
    const pT = clamp01(scrollY / Math.max(span, 1));
    pS += (pT - pS) * Math.min(1, dt * 6.5);
    if (!hintHidden && pT > 0.004) {
      hintHidden = true;
      $("#hint").classList.remove("show");
    }
    world.update(pS, dt, now / 1000);
    hud(pS, dt);
    drawCharts(now);
    requestAnimationFrame(frame);
  })(last);
})();

/* rail navigation */
railDots.forEach((d) =>
  d.addEventListener("click", () => {
    const r = ranges.find((x) => x.id === d.dataset.act);
    if (!r) return;
    const span = document.documentElement.scrollHeight - innerHeight;
    scrollTo({ top: r.start * span + 4, behavior: REDUCED ? "auto" : "smooth" });
  })
);
