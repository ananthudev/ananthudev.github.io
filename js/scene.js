/* ============================================================
   Mission-Control HUD — 3D scene (Three.js)
   All airframes are generic & illustrative.
   ============================================================ */
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const canvas = document.getElementById("scene");
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
const MOBILE = matchMedia("(max-width: 860px)").matches;

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: !MOBILE, alpha: true });
} catch (e) {
  document.body.classList.add("no-webgl");
}

const AMBER = 0xffab40;
const AMBER_HI = 0xffc46e;

if (renderer) {
  renderer.setPixelRatio(Math.min(devicePixelRatio, MOBILE ? 1.5 : 2));
  renderer.setSize(innerWidth, innerHeight);

  const scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x060709, 0.016);

  const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 400);
  camera.position.set(6, 3.2, 11);
  const camTarget = new THREE.Vector3(0, 2, 0);

  scene.add(new THREE.AmbientLight(0x404040, 1.2));
  const key = new THREE.DirectionalLight(0xffd9a0, 1.4);
  key.position.set(6, 10, 6);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x6688ff, 0.5);
  rim.position.set(-8, 4, -6);
  scene.add(rim);

  /* ---------- Materials ---------- */
  const matWire = (c = AMBER, op = 0.5) =>
    new THREE.MeshBasicMaterial({ color: c, wireframe: true, transparent: true, opacity: op });
  const matHull = (c = 0x1a1d20) =>
    new THREE.MeshStandardMaterial({ color: c, metalness: 0.85, roughness: 0.35, emissive: 0x0a0702, emissiveIntensity: 0.4 });
  const matGlow = (c = AMBER, op = 1) =>
    new THREE.MeshBasicMaterial({ color: c, transparent: op < 1, opacity: op });

  function holo(geo, hullColor) {
    const g = new THREE.Group();
    g.add(new THREE.Mesh(geo, matHull(hullColor)));
    const w = new THREE.Mesh(geo, matWire(AMBER, 0.22));
    w.scale.setScalar(1.012);
    g.add(w);
    return g;
  }

  /* ---------- Environment: stars, grid, radar ---------- */
  const starGeo = new THREE.BufferGeometry();
  const N_STARS = MOBILE ? 900 : 2200;
  const starPos = new Float32Array(N_STARS * 3);
  for (let i = 0; i < N_STARS; i++) {
    const r = 90 + Math.random() * 160;
    const th = Math.random() * Math.PI * 2;
    const ph = Math.acos(2 * Math.random() - 1);
    starPos[i * 3] = r * Math.sin(ph) * Math.cos(th);
    starPos[i * 3 + 1] = Math.abs(r * Math.cos(ph)) * 0.7 - 6;
    starPos[i * 3 + 2] = r * Math.sin(ph) * Math.sin(th);
  }
  starGeo.setAttribute("position", new THREE.BufferAttribute(starPos, 3));
  const stars = new THREE.Points(starGeo, new THREE.PointsMaterial({ color: 0xcfd4da, size: 0.35, sizeAttenuation: true, transparent: true, opacity: 0.8 }));
  scene.add(stars);

  const grid = new THREE.GridHelper(240, 120, AMBER, 0x2a2416);
  grid.material.transparent = true;
  grid.material.opacity = 0.28;
  scene.add(grid);

  // radar sweep on the ground
  const radarGroup = new THREE.Group();
  for (const r of [4, 8, 12]) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(r - 0.02, r, 96),
      matGlow(AMBER, 0.14)
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.01;
    radarGroup.add(ring);
  }
  const sweepGeo = new THREE.CircleGeometry(12, 48, 0, Math.PI / 5);
  const sweepMat = new THREE.MeshBasicMaterial({ color: AMBER, transparent: true, opacity: 0.07, side: THREE.DoubleSide });
  const sweep = new THREE.Mesh(sweepGeo, sweepMat);
  sweep.rotation.x = -Math.PI / 2;
  sweep.position.y = 0.02;
  radarGroup.add(sweep);
  scene.add(radarGroup);

  /* ---------- Model builders (generic, illustrative) ---------- */
  function buildInterceptor() {
    const g = new THREE.Group();
    const body = new THREE.Group();
    // nose cone (ogive-ish via lathe)
    const pts = [];
    for (let i = 0; i <= 14; i++) {
      const t = i / 14;
      pts.push(new THREE.Vector2(0.42 * Math.cos(t * Math.PI * 0.5), 1.6 * t));
    }
    const nose = holo(new THREE.LatheGeometry(pts, 28), 0x23262a);
    nose.position.y = 3.2;
    body.add(nose);
    // main body
    const tube = holo(new THREE.CylinderGeometry(0.42, 0.46, 3.3, 28), 0x1a1d20);
    tube.position.y = 1.55;
    body.add(tube);
    // stripes
    for (const y of [2.6, 0.7]) {
      const band = new THREE.Mesh(new THREE.CylinderGeometry(0.435, 0.435, 0.1, 28), matGlow(AMBER, 0.9));
      band.position.y = y;
      body.add(band);
    }
    // fins (4 tail + 4 canard)
    const finShape = new THREE.Shape();
    finShape.moveTo(0, 0); finShape.lineTo(0.75, -0.1); finShape.lineTo(0.75, -0.42); finShape.lineTo(0, -0.95); finShape.closePath();
    const finGeo = new THREE.ExtrudeGeometry(finShape, { depth: 0.04, bevelEnabled: false });
    for (let i = 0; i < 4; i++) {
      const fin = holo(finGeo, 0x23262a);
      fin.position.y = 0.95;
      fin.rotation.y = (i * Math.PI) / 2;
      fin.children.forEach(m => m.position.x = 0.4);
      body.add(fin);
      const can = holo(finGeo, 0x23262a);
      can.scale.setScalar(0.45);
      can.position.y = 3.1;
      can.rotation.y = (i * Math.PI) / 2 + Math.PI / 4;
      can.children.forEach(m => m.position.x = 0.9);
      body.add(can);
    }
    // nozzle
    const noz = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.38, 0.3, 24, 1, true), matHull(0x111315));
    noz.position.y = -0.15;
    body.add(noz);
    const flame = new THREE.Mesh(new THREE.ConeGeometry(0.26, 1.4, 20, 1, true), matGlow(0xffc46e, 0.85));
    flame.rotation.x = Math.PI;
    flame.position.y = -0.95;
    flame.name = "flame";
    body.add(flame);
    const flameCore = new THREE.Mesh(new THREE.ConeGeometry(0.13, 0.9, 16, 1, true), matGlow(0xffffff, 0.9));
    flameCore.rotation.x = Math.PI;
    flameCore.position.y = -0.7;
    flameCore.name = "flameCore";
    body.add(flameCore);
    g.add(body);
    g.userData.body = body;
    return g;
  }

  function buildLauncher() {
    const g = new THREE.Group();
    const core = holo(new THREE.CylinderGeometry(0.6, 0.66, 5.2, 28), 0x20242a);
    core.position.y = 2.6;
    g.add(core);
    const pts = [];
    for (let i = 0; i <= 12; i++) {
      const t = i / 12;
      pts.push(new THREE.Vector2(0.6 * Math.cos(t * Math.PI * 0.5), 1.1 * t));
    }
    const nose = holo(new THREE.LatheGeometry(pts, 28), 0x20242a);
    nose.position.y = 5.2;
    g.add(nose);
    for (let i = 0; i < 2; i++) {
      const b = holo(new THREE.CylinderGeometry(0.26, 0.3, 3.4, 20), 0x1a1d20);
      b.position.set(i ? 0.95 : -0.95, 1.7, 0);
      g.add(b);
      const bn = holo(new THREE.ConeGeometry(0.26, 0.6, 20), 0x1a1d20);
      bn.position.set(i ? 0.95 : -0.95, 3.7, 0);
      g.add(bn);
    }
    const band = new THREE.Mesh(new THREE.CylinderGeometry(0.615, 0.615, 0.14, 28), matGlow(AMBER, 0.9));
    band.position.y = 4.4;
    g.add(band);
    return g;
  }

  function buildSatellite() {
    const g = new THREE.Group();
    const bus = holo(new THREE.BoxGeometry(1.1, 1.3, 1.1), 0x22262c);
    bus.position.y = 2.6;
    g.add(bus);
    for (const s of [-1, 1]) {
      const wing = new THREE.Group();
      const panel = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.06, 1.1), new THREE.MeshStandardMaterial({ color: 0x14202e, metalness: 0.6, roughness: 0.3, emissive: 0x0a1a2e, emissiveIntensity: 0.6 }));
      panel.position.x = s * 2.0;
      wing.add(panel);
      const wire = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.062, 1.1), matWire(0x4a9fff, 0.35));
      wire.position.x = s * 2.0;
      wing.add(wire);
      wing.position.y = 2.6;
      g.add(wing);
    }
    const dish = holo(new THREE.SphereGeometry(0.45, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2.6), 0x2a2e34);
    dish.rotation.x = Math.PI;
    dish.position.y = 3.6;
    g.add(dish);
    const ant = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.8), matGlow(AMBER));
    ant.position.y = 3.6;
    g.add(ant);
    return g;
  }

  function buildUGV() {
    const g = new THREE.Group();
    const hullGeo = new THREE.BoxGeometry(2.6, 0.7, 1.5);
    const hull = holo(hullGeo, 0x1d211d);
    hull.position.y = 0.75;
    g.add(hull);
    const glacis = holo(new THREE.BoxGeometry(0.9, 0.5, 1.5), 0x1d211d);
    glacis.position.set(1.55, 0.62, 0);
    glacis.rotation.z = -0.5;
    g.add(glacis);
    const turret = holo(new THREE.CylinderGeometry(0.55, 0.65, 0.42, 8), 0x23272a);
    turret.position.y = 1.35;
    g.add(turret);
    const barrel = holo(new THREE.CylinderGeometry(0.07, 0.09, 1.9, 12), 0x23272a);
    barrel.rotation.z = Math.PI / 2 - 0.08;
    barrel.position.set(1.4, 1.42, 0);
    g.add(barrel);
    for (let i = 0; i < 4; i++) {
      for (const s of [-1, 1]) {
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 0.22, 18), matHull(0x101210));
        wheel.rotation.x = Math.PI / 2;
        wheel.position.set(-1.0 + i * 0.68, 0.32, s * 0.78);
        g.add(wheel);
        const rim2 = new THREE.Mesh(new THREE.TorusGeometry(0.3, 0.025, 8, 24), matGlow(AMBER, 0.5));
        rim2.position.copy(wheel.position);
        g.add(rim2);
      }
    }
    g.scale.setScalar(1.15);
    return g;
  }

  /* ---------- Hangar assembly ---------- */
  const models = {
    interceptor: buildInterceptor(),
    launcher: buildLauncher(),
    satellite: buildSatellite(),
    ugv: buildUGV(),
  };
  const heroGroup = new THREE.Group();
  for (const [k, m] of Object.entries(models)) {
    m.visible = k === "interceptor";
    heroGroup.add(m);
  }
  scene.add(heroGroup);
  let activeModel = "interceptor";

  // Optional user-supplied GLB replaces the procedural interceptor
  new GLTFLoader().load("models/hero.glb", (gltf) => {
    const m = gltf.scene;
    const box = new THREE.Box3().setFromObject(m);
    const size = box.getSize(new THREE.Vector3());
    const s = 4.5 / Math.max(size.x, size.y, size.z);
    m.scale.setScalar(s);
    box.setFromObject(m);
    m.position.y -= box.min.y;
    models.interceptor.visible = false;
    heroGroup.add(m);
    models.interceptor = m;
    if (activeModel === "interceptor") m.visible = true;
    else m.visible = false;
  }, undefined, () => { /* no GLB provided — procedural hero stays */ });

  /* ---------- Earth + orbit (Simulation) ---------- */
  const simGroup = new THREE.Group();
  const earth = new THREE.Group();
  earth.add(new THREE.Mesh(new THREE.SphereGeometry(3, 28, 20), new THREE.MeshStandardMaterial({ color: 0x0a1626, metalness: 0.1, roughness: 0.9, transparent: true, opacity: 0.92 })));
  earth.add(new THREE.Mesh(new THREE.SphereGeometry(3.005, 28, 20), matWire(0x3f87d4, 0.35)));
  simGroup.add(earth);
  const orbitCurve = new THREE.EllipseCurve(0, 0, 4.6, 3.9, 0, Math.PI * 2);
  const orbitPts = orbitCurve.getPoints(120).map(p => new THREE.Vector3(p.x, 0, p.y));
  const orbitLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(orbitPts), new THREE.LineBasicMaterial({ color: AMBER, transparent: true, opacity: 0.7 }));
  orbitLine.rotation.x = 0.45;
  simGroup.add(orbitLine);
  const satDot = new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8), matGlow(AMBER));
  simGroup.add(satDot);
  simGroup.position.set(0, 3.2, 0);
  simGroup.visible = false;
  scene.add(simGroup);

  /* ---------- GNC intercept visual ---------- */
  const gncGroup = new THREE.Group();
  // target: ballistic arc right→left; interceptor: rises from origin, curves to meet
  const tgtPts = [], intPts = [];
  const APEX = 9, SPAN = 16;
  const hit = new THREE.Vector3(-2.5, 6.2, -2);
  for (let i = 0; i <= 100; i++) {
    const t = i / 100;
    tgtPts.push(new THREE.Vector3(SPAN / 2 - t * (SPAN / 2 - hit.x), APEX * (1 - Math.pow(1.6 * t - 0.6, 2)) * 0.9 + t * hit.y * 0.4, -6 + t * (hit.z + 6)));
  }
  const intStart = new THREE.Vector3(-7, 0, 3);
  const intCtl = new THREE.Vector3(-5.5, 5.5, 0.5);
  const intCurve = new THREE.QuadraticBezierCurve3(intStart, intCtl, hit);
  for (let i = 0; i <= 100; i++) intPts.push(intCurve.getPoint(i / 100));
  const tgtLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(tgtPts), new THREE.LineBasicMaterial({ color: 0xff5050, transparent: true, opacity: 0.85 }));
  const intLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(intPts), new THREE.LineBasicMaterial({ color: AMBER_HI, transparent: true, opacity: 0.95 }));
  tgtLine.geometry.setDrawRange(0, 0);
  intLine.geometry.setDrawRange(0, 0);
  gncGroup.add(tgtLine, intLine);
  const tgtMk = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.5, 10), matGlow(0xff5050));
  const intMk = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.5, 10), matGlow(AMBER_HI));
  const burst = new THREE.Mesh(new THREE.SphereGeometry(0.1, 14, 10), matGlow(0xffffff, 0.95));
  burst.visible = false;
  gncGroup.add(tgtMk, intMk, burst);
  gncGroup.visible = false;
  scene.add(gncGroup);
  let gncT = 0, gncPlaying = false;

  function playIntercept() {
    gncT = 0;
    gncPlaying = true;
    burst.visible = false;
    burst.scale.setScalar(1);
  }

  /* ---------- Exhaust particles ---------- */
  const N_EX = MOBILE ? 120 : 300;
  const exGeo = new THREE.BufferGeometry();
  const exPos = new Float32Array(N_EX * 3);
  const exLife = new Float32Array(N_EX);
  for (let i = 0; i < N_EX; i++) exLife[i] = Math.random();
  exGeo.setAttribute("position", new THREE.BufferAttribute(exPos, 3));
  const exhaust = new THREE.Points(exGeo, new THREE.PointsMaterial({ color: 0xffb060, size: 0.08, transparent: true, opacity: 0.7 }));
  scene.add(exhaust);

  /* ---------- Section choreography ---------- */
  const CAMS = {
    ops:    { pos: [6, 3.2, 11],   tgt: [0, 2.2, 0],  heroX: 4.6,  hero: true,  sim: false, gnc: false },
    fsw:    { pos: [-5, 2.6, 9],   tgt: [0, 2.4, 0],  heroX: 1.2,  hero: true,  sim: false, gnc: false },
    gnc:    { pos: [-2, 5, 16.5],  tgt: [0.5, 4.5, 0], heroX: 0,   hero: false, sim: false, gnc: true  },
    sim:    { pos: [-6, 5.5, 10],  tgt: [0, 3.2, 0],  heroX: 0,    hero: false, sim: true,  gnc: false },
    tools:  { pos: [7, 2.2, 9],    tgt: [0, 2, 0],    heroX: -1.5, hero: true,  sim: false, gnc: false },
    vnv:    { pos: [-7, 3, 8],     tgt: [0, 2, 0],    heroX: 1.5,  hero: true,  sim: false, gnc: false },
    stack:  { pos: [0, 8, 12],     tgt: [0, 1, 0],    heroX: -2.5, hero: true,  sim: false, gnc: false },
    hangar: { pos: [6.5, 3.4, 11], tgt: [0, 2.6, 0],  heroX: 0,    hero: true,  sim: false, gnc: false },
    field:  { pos: [3, 2.5, 12],   tgt: [0, 2, 0],    heroX: -3,   hero: true,  sim: false, gnc: false },
    comms:  { pos: [0, 2.2, 13],   tgt: [0, 2.5, 0],  heroX: 4,    hero: true,  sim: false, gnc: false },
  };
  const heroShift = { x: CAMS.ops.heroX };
  let currentModule = "ops";

  function setModule(name) {
    if (!CAMS[name] || name === currentModule) return;
    currentModule = name;
    const c = CAMS[name];
    heroGroup.visible = c.hero;
    simGroup.visible = c.sim;
    gncGroup.visible = c.gnc;
    radarGroup.visible = name !== "sim";
    if (name === "gnc") playIntercept();
    if (window.gsap && !REDUCED) {
      gsap.to(camera.position, { x: c.pos[0], y: c.pos[1], z: c.pos[2], duration: 1.6, ease: "power2.inOut" });
      gsap.to(camTarget, { x: c.tgt[0], y: c.tgt[1], z: c.tgt[2], duration: 1.6, ease: "power2.inOut" });
      gsap.to(heroShift, { x: c.heroX, duration: 1.6, ease: "power2.inOut" });
    } else {
      camera.position.set(...c.pos);
      camTarget.set(...c.tgt);
      heroShift.x = c.heroX;
    }
  }

  function setHangarModel(name) {
    if (!models[name]) return;
    activeModel = name;
    for (const [k, m] of Object.entries(models)) m.visible = k === name;
  }

  let phase = 0;
  function setPhase(i) { phase = i; }

  window.HUD_SCENE = { setModule, setHangarModel, setPhase, playIntercept };
  window.dispatchEvent(new Event("hud-scene-ready"));

  /* ---------- Hangar drag-orbit ---------- */
  let dragging = false, px = 0, py = 0, yaw = 0, pitch = 0;
  canvas.style.pointerEvents = "none";
  // listen on window so panels don't block casual drags in hangar section
  addEventListener("pointerdown", (e) => {
    if (currentModule !== "hangar" || e.target.closest(".panel, #navrail, #topbar, a, button")) return;
    dragging = true; px = e.clientX; py = e.clientY;
  });
  addEventListener("pointermove", (e) => {
    if (!dragging) return;
    yaw += (e.clientX - px) * 0.006;
    pitch = Math.max(-0.5, Math.min(0.9, pitch + (e.clientY - py) * 0.004));
    px = e.clientX; py = e.clientY;
  });
  addEventListener("pointerup", () => (dragging = false));

  /* ---------- Resize ---------- */
  addEventListener("resize", () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  /* ---------- Animation loop ---------- */
  const clock = new THREE.Clock();
  let t = 0;

  function animate() {
    const dt = Math.min(clock.getDelta(), 0.05);
    if (!REDUCED) t += dt;

    // hero idle motion
    const hero = models[activeModel];
    if (hero && heroGroup.visible) {
      const launchish = phase >= 2 && activeModel === "interceptor" && currentModule === "fsw";
      heroGroup.position.x = heroShift.x;
      heroGroup.position.y = (launchish ? 1.6 : 0) + Math.sin(t * 0.8) * 0.15;
      heroGroup.rotation.y = yaw + (currentModule === "hangar" ? 0 : t * 0.15);
      heroGroup.rotation.x = currentModule === "hangar" ? pitch * 0.5 : 0;
      const flame = hero.getObjectByName?.("flame");
      const flameCore = hero.getObjectByName?.("flameCore");
      if (flame) {
        const on = phase >= 2 || currentModule !== "fsw";
        flame.visible = flameCore.visible = on && activeModel === "interceptor";
        if (on) {
          flame.scale.y = 1 + Math.sin(t * 30) * 0.18 + Math.random() * 0.1;
          flameCore.scale.y = 1 + Math.sin(t * 40) * 0.25;
        }
      }
    }

    // exhaust particles under interceptor
    if (heroGroup.visible && activeModel === "interceptor") {
      exhaust.visible = true;
      const base = heroGroup.position.y - 0.4;
      for (let i = 0; i < N_EX; i++) {
        exLife[i] += dt * (0.9 + (i % 7) * 0.08);
        if (exLife[i] > 1) exLife[i] = 0;
        const l = exLife[i];
        const ang = i * 2.399;
        exPos[i * 3] = Math.cos(ang + t) * 0.2 * (1 + l * 2.2);
        exPos[i * 3 + 1] = base - l * 4.2;
        exPos[i * 3 + 2] = Math.sin(ang + t) * 0.2 * (1 + l * 2.2);
      }
      exGeo.attributes.position.needsUpdate = true;
      exhaust.material.opacity = 0.65 * (1 - 0.3 * Math.sin(t * 7));
      exhaust.rotation.y = heroGroup.rotation.y;
      exhaust.position.copy(heroGroup.position).setY(0);
    } else exhaust.visible = false;

    // radar sweep
    sweep.rotation.z = t * 0.9;

    // sim: earth spin + satellite
    if (simGroup.visible) {
      earth.rotation.y = t * 0.25;
      const p = orbitCurve.getPoint((t * 0.07) % 1);
      satDot.position.set(p.x, 0, p.y).applyAxisAngle(new THREE.Vector3(1, 0, 0), 0.45);
    }

    // gnc intercept playback
    if (gncGroup.visible && gncPlaying) {
      gncT += dt * 0.22;
      const k = Math.min(gncT, 1);
      const n = Math.floor(k * 100);
      tgtLine.geometry.setDrawRange(0, n);
      intLine.geometry.setDrawRange(0, n);
      tgtMk.position.copy(tgtPts[Math.min(n, 100)]);
      intMk.position.copy(intPts[Math.min(n, 100)]);
      if (n > 2) {
        tgtMk.lookAt(tgtPts[n - 2]); tgtMk.rotateX(-Math.PI / 2);
        intMk.lookAt(intPts[n - 2]); intMk.rotateX(Math.PI / 2);
      }
      if (k >= 1) {
        burst.visible = true;
        burst.position.copy(hit);
        burst.scale.multiplyScalar(1 + dt * 14);
        burst.material.opacity = Math.max(0, burst.material.opacity - dt * 1.2);
        if (burst.material.opacity <= 0.02) {
          gncPlaying = false;
          burst.material.opacity = 0.95;
          burst.visible = false;
          tgtMk.visible = intMk.visible = false;
          // auto-loop while the GNC module is on screen
          if (!REDUCED) setTimeout(() => { if (currentModule === "gnc") playIntercept(); }, 2500);
        }
      } else {
        tgtMk.visible = intMk.visible = true;
      }
    }

    // stars drift
    stars.rotation.y = t * 0.004;

    camera.lookAt(camTarget);
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }
  animate();
}
