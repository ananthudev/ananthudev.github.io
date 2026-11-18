/* ============================================================
   THE RANGE — director: scroll, chapters, sound, panels
   ============================================================ */
import { createWorld } from "./world.js";

const $ = (s, c = document) => c.querySelector(s);
const $$ = (s, c = document) => [...c.querySelectorAll(s)];
const clamp01 = (x) => Math.min(1, Math.max(0, x));
const lerp = (a, b, k) => a + (b - a) * k;
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
const MOBILE = matchMedia("(max-width: 860px)").matches;

$("#year").textContent = new Date().getFullYear();

/* ---------- sound (synthesized, off by default) ---------- */
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
    const buf = this.ctx.createBuffer(1, this.ctx.sampleRate * 2, this.ctx.sampleRate);
    const d = buf.getChannelData(0);
    let last = 0;
    for (let i = 0; i < d.length; i++) {
      const w = Math.random() * 2 - 1;
      last = (last + 0.02 * w) / 1.02;
      d[i] = last * 3.2;
    }
    this.noiseBuf = buf;
    this.wind = this.loop(buf, "bandpass", 420, 0);
    this.rumble = this.loop(buf, "lowpass", 70, 0);
  }
  loop(buf, type, freq, gain) {
    const src = this.ctx.createBufferSource();
    src.buffer = buf; src.loop = true;
    const f = this.ctx.createBiquadFilter();
    f.type = type; f.frequency.value = freq; f.Q.value = 0.4;
    const g = this.ctx.createGain();
    g.gain.value = gain;
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start();
    return g;
  }
  setAmbient(wind, rumble, dt) {
    if (!this.ctx) return;
    const k = 1 - Math.pow(0.02, dt);
    this.wind.gain.value = lerp(this.wind.gain.value, wind, k);
    this.rumble.gain.value = lerp(this.rumble.gain.value, rumble, k);
  }
  boom(freq = 55, dur = 2.4, peak = 1.1) { // artillery
    if (!this.ctx || !this.on) return;
    const t0 = this.ctx.currentTime;
    const src = this.ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const f = this.ctx.createBiquadFilter();
    f.type = "lowpass";
    f.frequency.setValueAtTime(900, t0);
    f.frequency.exponentialRampToValueAtTime(freq, t0 + dur * 0.7);
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(peak, t0);
    g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start(t0); src.stop(t0 + dur + 0.1);
    const o = this.ctx.createOscillator();
    o.type = "sine";
    o.frequency.setValueAtTime(38, t0);
    o.frequency.exponentialRampToValueAtTime(24, t0 + dur);
    const og = this.ctx.createGain();
    og.gain.setValueAtTime(peak * 0.9, t0);
    og.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    o.connect(og); og.connect(this.master);
    o.start(t0); o.stop(t0 + dur + 0.1);
  }
  whoosh(dur = 1.2, peak = 0.35) {
    if (!this.ctx || !this.on) return;
    const t0 = this.ctx.currentTime;
    const src = this.ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const f = this.ctx.createBiquadFilter();
    f.type = "bandpass"; f.Q.value = 1.2;
    f.frequency.setValueAtTime(300, t0);
    f.frequency.exponentialRampToValueAtTime(2600, t0 + dur * 0.4);
    f.frequency.exponentialRampToValueAtTime(240, t0 + dur);
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(0.001, t0);
    g.gain.exponentialRampToValueAtTime(peak, t0 + dur * 0.35);
    g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start(t0); src.stop(t0 + dur + 0.1);
  }
  click(freq = 1100, dur = 0.05, vol = 0.05) {
    if (!this.ctx || !this.on) return;
    const t0 = this.ctx.currentTime;
    const o = this.ctx.createOscillator();
    o.type = "triangle"; o.frequency.value = freq;
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(vol, t0);
    g.gain.exponentialRampToValueAtTime(0.001, t0 + dur);
    o.connect(g); g.connect(this.master);
    o.start(t0); o.stop(t0 + dur + 0.03);
  }
  toggle() {
    this.ensure();
    if (!this.ctx) return false;
    if (this.ctx.state === "suspended") this.ctx.resume();
    this.on = !this.on;
    this.master.gain.linearRampToValueAtTime(this.on ? 0.5 : 0, this.ctx.currentTime + 0.4);
    return this.on;
  }
}
const snd = new SoundEngine();
const sndBtn = $("#sndBtn");
sndBtn.addEventListener("click", () => {
  const on = snd.toggle();
  sndBtn.setAttribute("aria-pressed", on);
  sndBtn.textContent = on ? "SOUND ON" : "SOUND OFF";
});

/* ---------- panel reveal ---------- */
const reveal = new IntersectionObserver(
  (es) => es.forEach((e) => e.isIntersecting && e.target.classList.add("revealed")),
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
if (charts[0]) new IntersectionObserver((es) => es.forEach((e) => (chartsLive = e.isIntersecting)), { threshold: 0.1 }).observe(charts[0]);
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
    x.strokeStyle = "rgba(255,255,255,0.12)";
    x.lineWidth = 1;
    for (let gy = 0; gy <= h; gy += h / 3) { x.beginPath(); x.moveTo(0, gy); x.lineTo(w, gy); x.stroke(); }
    const kind = c.dataset.kind;
    x.strokeStyle = "#e8e4dc"; x.lineWidth = 1.3;
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
  }
}

/* ---------- film bootstrap ---------- */
const canvas = $("#film");
const loader = $("#loader");
const acts = $$("section.act");
const railDots = $$("#rail button");
const actLabel = $("#actlabel");
const calloutEl = $("#callout");
const cut = $("#cut");

const CUTS = new Set([
  "firemission>tel", "tel>firemission", "tel>jet", "jet>tel", "jet>mlrs", "mlrs>jet",
  "mlrs>night", "night>mlrs", "night>studio", "studio>night", "studio>comms", "comms>studio",
]);
const LETTERBOX = new Set(["firemission", "tel", "jet", "mlrs"]);
const ACT_NAMES = {
  title: "THE RANGE — DUSK", operator: "CH·01 THE OPERATOR", firemission: "CH·02 FIRE MISSION",
  tel: "CH·03 GUIDANCE", jet: "CH·04 FLIGHT TEST", mlrs: "CH·05 TRAJECTORIES",
  night: "CH·06 NIGHT WATCH", studio: "CH·07 THE ARSENAL", comms: "CH·08 END CARD",
};
const AMBIENT = { // [wind, rumble]
  title: [0.05, 0.04], operator: [0.05, 0.05], firemission: [0.045, 0.06], tel: [0.04, 0.06],
  jet: [0.05, 0.1], mlrs: [0.04, 0.07], night: [0.03, 0.03], studio: [0.008, 0.02], comms: [0.03, 0.03],
};
const CALLOUTS = [
  { act: "firemission", q: 0.1, text: "ELEVATION SET", snd: () => snd.click(900) },
  { act: "firemission", q: 0.3, text: "GUN READY", snd: () => snd.click(1200) },
  { act: "firemission", q: 0.342, text: "FIRE", big: true, snd: () => snd.boom(55, 2.6, 1.15) },
  { act: "firemission", q: 0.872, text: "SPLASH · TARGET", snd: () => snd.boom(70, 1.8, 0.7) },
  { act: "tel", q: 0.12, text: "ERECTOR UP", snd: () => snd.click(800) },
  { act: "tel", q: 0.5, text: "GNC ALIGNED", snd: () => snd.click(1100) },
  { act: "tel", q: 0.625, text: "MISSILE AWAY", big: true, snd: () => snd.boom(80, 2.2, 0.9) },
  { act: "jet", q: 0.42, text: "LOW PASS", snd: () => snd.whoosh(1.6, 0.5) },
  { act: "mlrs", q: 0.225, text: "RIPPLE FIRE", big: true, snd: () => { snd.boom(90, 1.2, 0.55); snd.whoosh(1.1, 0.3); } },
  { act: "mlrs", q: 0.5, text: "SIX AWAY", snd: () => snd.whoosh(1, 0.25) },
  { act: "night", q: 0.25, text: "NIGHT WATCH", snd: () => snd.click(700, 0.08) },
  { act: "comms", q: 0.15, text: "END OF EXERCISE", snd: () => snd.click(600, 0.1) },
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
      setTimeout(() => cut.classList.remove("dip"), 400);
    }
    prevAct = id;
    actLabel.textContent = ACT_NAMES[id] || id.toUpperCase();
    railDots.forEach((d) => d.classList.toggle("active", d.dataset.act === id));
    document.body.classList.toggle("letterbox", LETTERBOX.has(id));
  }

  CALLOUTS.forEach((c, i) => {
    const inAct = c.act === id;
    if (inAct && q >= c.q && armed[i]) {
      armed[i] = false;
      calloutEl.textContent = c.text;
      calloutEl.classList.toggle("big", !!c.big);
      calloutEl.classList.add("show");
      calloutTimer = c.big ? 1.4 : 2.0;
      c.snd && c.snd();
    }
    if (inAct && q < c.q - 0.04) armed[i] = true;
    if (!inAct && c.act !== prevAct) armed[i] = true;
  });
  if (calloutTimer > 0) {
    calloutTimer -= dt;
    if (calloutTimer <= 0) calloutEl.classList.remove("show");
  }

  const amb = AMBIENT[id] || [0.04, 0.04];
  snd.setAmbient(snd.on ? amb[0] : 0, snd.on ? amb[1] : 0, dt);
}

(async () => {
  world = await createWorld(canvas, { tier: MOBILE ? "low" : "hi" });
  window.__world = world;
  if (!world) {
    document.body.classList.add("no-webgl");
    loader.classList.add("done");
    setTimeout(() => loader.remove(), 800);
    return;
  }
  measure();
  addEventListener("resize", () => { measure(); world.resize(); });

  addEventListener("pointermove", (e) => {
    world.pointer.x = (e.clientX / innerWidth) * 2 - 1;
    world.pointer.y = (e.clientY / innerHeight) * 2 - 1;
  });

  loader.classList.add("done");
  setTimeout(() => loader.remove(), 900);
  $("#hint").classList.add("show");
  let hintHidden = false;

  if (REDUCED) {
    world.update(0.004, 0.016, 0.4);
    document.body.classList.add("reduced");
    return;
  }

  // auto-degrade on sustained low FPS (once)
  let slowTime = 0, degraded = false;
  let last = performance.now();
  (function frame(now) {
    const rawDt = (now - last) / 1000;
    const dt = Math.min(rawDt, 0.05);
    last = now;
    if (!degraded) {
      slowTime = rawDt > 0.045 ? slowTime + rawDt : 0;
      if (slowTime > 3) { degraded = true; world.degrade(); }
    }
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

railDots.forEach((d) =>
  d.addEventListener("click", () => {
    const r = ranges.find((x) => x.id === d.dataset.act);
    if (!r) return;
    const span = document.documentElement.scrollHeight - innerHeight;
    scrollTo({ top: r.start * span + 4, behavior: REDUCED ? "auto" : "smooth" });
  })
);
