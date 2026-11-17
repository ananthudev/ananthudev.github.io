/* ============================================================
   Mission Film — model & material builders
   All airframes generic & illustrative.
   ============================================================ */
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export const AMBER = 0xffab40;
export const AMBER_HI = 0xffc46e;

export const matWire = (c = AMBER, op = 0.5) =>
  new THREE.MeshBasicMaterial({ color: c, wireframe: true, transparent: true, opacity: op });
export const matHull = (c = 0x1a1d20) =>
  new THREE.MeshStandardMaterial({ color: c, metalness: 0.85, roughness: 0.35, emissive: 0x0a0702, emissiveIntensity: 0.4 });
export const matGlow = (c = AMBER, op = 1) =>
  new THREE.MeshBasicMaterial({ color: c, transparent: op < 1, opacity: op });

export function holo(geo, hullColor) {
  const g = new THREE.Group();
  g.add(new THREE.Mesh(geo, matHull(hullColor)));
  const w = new THREE.Mesh(geo, matWire(AMBER, 0.2));
  w.scale.setScalar(1.012);
  g.add(w);
  return g;
}

/* ---------- Procedural interceptor (fallback + hangar) ---------- */
export function buildInterceptor() {
  const g = new THREE.Group();
  const pts = [];
  for (let i = 0; i <= 14; i++) {
    const t = i / 14;
    pts.push(new THREE.Vector2(0.42 * Math.cos(t * Math.PI * 0.5), 1.6 * t));
  }
  const nose = holo(new THREE.LatheGeometry(pts, 28), 0x23262a);
  nose.position.y = 3.2;
  g.add(nose);
  const tube = holo(new THREE.CylinderGeometry(0.42, 0.46, 3.3, 28), 0x1a1d20);
  tube.position.y = 1.55;
  g.add(tube);
  for (const y of [2.6, 0.7]) {
    const band = new THREE.Mesh(new THREE.CylinderGeometry(0.435, 0.435, 0.1, 28), matGlow(AMBER, 0.9));
    band.position.y = y;
    g.add(band);
  }
  const finShape = new THREE.Shape();
  finShape.moveTo(0, 0); finShape.lineTo(0.75, -0.1); finShape.lineTo(0.75, -0.42); finShape.lineTo(0, -0.95); finShape.closePath();
  const finGeo = new THREE.ExtrudeGeometry(finShape, { depth: 0.04, bevelEnabled: false });
  for (let i = 0; i < 4; i++) {
    const fin = holo(finGeo, 0x23262a);
    fin.position.y = 0.95;
    fin.rotation.y = (i * Math.PI) / 2;
    fin.children.forEach(m => (m.position.x = 0.4));
    g.add(fin);
    const can = holo(finGeo, 0x23262a);
    can.scale.setScalar(0.45);
    can.position.y = 3.1;
    can.rotation.y = (i * Math.PI) / 2 + Math.PI / 4;
    can.children.forEach(m => (m.position.x = 0.9));
    g.add(can);
  }
  return g;
}

export function buildLauncher() {
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

export function buildSatellite() {
  const g = new THREE.Group();
  const bus = holo(new THREE.BoxGeometry(1.1, 1.3, 1.1), 0x22262c);
  bus.position.y = 0;
  g.add(bus);
  const pivots = [];
  for (const s of [-1, 1]) {
    const pivot = new THREE.Group();
    pivot.position.set(s * 0.55, 0, 0);
    const wing = new THREE.Group();
    const panel = new THREE.Mesh(
      new THREE.BoxGeometry(2.6, 0.06, 1.1),
      new THREE.MeshStandardMaterial({ color: 0x14202e, metalness: 0.6, roughness: 0.3, emissive: 0x0a1a2e, emissiveIntensity: 0.6 })
    );
    panel.position.x = s * 1.42;
    wing.add(panel);
    const wire = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.062, 1.1), matWire(0x4a9fff, 0.35));
    wire.position.x = s * 1.42;
    wing.add(wire);
    pivot.add(wing);
    pivot.rotation.z = s * 1.5; // folded; unfold -> 0
    pivot.userData.side = s;
    g.add(pivot);
    pivots.push(pivot);
  }
  const dish = holo(new THREE.SphereGeometry(0.45, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2.6), 0x2a2e34);
  dish.rotation.x = Math.PI;
  dish.position.y = 1.0;
  g.add(dish);
  const ant = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.8), matGlow(AMBER));
  ant.position.y = 1.0;
  g.add(ant);
  g.userData.panelPivots = pivots;
  return g;
}

export function buildUGV() {
  const g = new THREE.Group();
  const hull = holo(new THREE.BoxGeometry(2.6, 0.7, 1.5), 0x1d211d);
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
      const rim = new THREE.Mesh(new THREE.TorusGeometry(0.3, 0.025, 8, 24), matGlow(AMBER, 0.5));
      rim.position.copy(wheel.position);
      g.add(rim);
    }
  }
  g.scale.setScalar(1.15);
  return g;
}

/* ---------- Gantry truss (LineSegments, one draw call) ---------- */
export function buildGantry(h = 6.4, w = 0.9) {
  const v = [];
  const seg = 0.8;
  const cs = [[-w, -w], [w, -w], [w, w], [-w, w]];
  for (let i = 0; i < 4; i++) {
    const [x1, z1] = cs[i], [x2, z2] = cs[(i + 1) % 4];
    v.push(x1, 0, z1, x1, h, z1); // verticals
    for (let y = seg; y <= h + 0.01; y += seg) {
      v.push(x1, y, z1, x2, y, z2); // rings
      const yPrev = y - seg;
      if (((y / seg) | 0) % 2 === i % 2) v.push(x1, yPrev, z1, x2, y, z2);
      else v.push(x2, yPrev, z2, x1, y, z1); // braces
    }
  }
  // crane arm at top
  v.push(-w, h, -w, -w - 1.6, h, -w, -w - 1.6, h, -w, -w - 1.6, h - 0.9, -w);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(v, 3));
  const lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: AMBER, transparent: true, opacity: 0.4 }));
  return lines;
}

/* ---------- Ground-station dish ---------- */
export function buildDish() {
  const g = new THREE.Group();
  const base = holo(new THREE.CylinderGeometry(0.7, 0.9, 0.5, 10), 0x1d2024);
  base.position.y = 0.25;
  g.add(base);
  const pedestal = holo(new THREE.CylinderGeometry(0.22, 0.3, 1.6, 10), 0x22262a);
  pedestal.position.y = 1.2;
  g.add(pedestal);
  const yoke = new THREE.Group();
  yoke.position.y = 2.0;
  const pts = [];
  for (let i = 0; i <= 12; i++) {
    const t = i / 12;
    pts.push(new THREE.Vector2(2.0 * t, 0.65 * t * t));
  }
  const bowlGeo = new THREE.LatheGeometry(pts, 26);
  const bowl = new THREE.Group();
  bowl.add(new THREE.Mesh(bowlGeo, new THREE.MeshStandardMaterial({ color: 0x242830, metalness: 0.7, roughness: 0.4, side: THREE.DoubleSide })));
  const bw = new THREE.Mesh(bowlGeo, matWire(AMBER, 0.3));
  bw.scale.setScalar(1.01);
  bowl.add(bw);
  const feed = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 1.5), matGlow(AMBER, 0.9));
  feed.position.y = 0.8;
  bowl.add(feed);
  const tip = new THREE.Mesh(new THREE.SphereGeometry(0.08, 10, 8), matGlow(AMBER_HI));
  tip.position.y = 1.55;
  bowl.add(tip);
  bowl.rotation.x = -0.9; // elevation
  yoke.add(bowl);
  g.add(yoke);
  g.userData.yoke = yoke;
  g.userData.bowl = bowl;
  return g;
}

/* ---------- Hero GLB loader (holo override + flame) ---------- */
export function loadHero(url) {
  return new Promise((resolve) => {
    new GLTFLoader().load(
      url,
      (gltf) => {
        const m = gltf.scene;
        const overlays = [];
        m.traverse((o) => {
          if (o.isMesh) {
            o.material = matHull(0x1e2125);
            const w = new THREE.Mesh(o.geometry, matWire(AMBER, 0.25));
            w.scale.setScalar(1.015);
            overlays.push([o, w]);
          }
        });
        for (const [o, w] of overlays) o.add(w);
        const box = new THREE.Box3().setFromObject(m);
        const size = box.getSize(new THREE.Vector3());
        const s = 4.8 / Math.max(size.x, size.y, size.z);
        const wrap = new THREE.Group();
        wrap.add(m);
        m.scale.setScalar(s);
        box.setFromObject(m);
        const c = box.getCenter(new THREE.Vector3());
        m.position.x -= c.x;
        m.position.z -= c.z;
        m.position.y -= box.min.y;
        const flame = new THREE.Mesh(new THREE.ConeGeometry(0.26, 1.5, 20, 1, true), matGlow(0xffc46e, 0.85));
        flame.rotation.x = Math.PI;
        flame.position.y = -0.78;
        flame.name = "flame";
        flame.visible = false;
        const flameCore = new THREE.Mesh(new THREE.ConeGeometry(0.13, 1.0, 16, 1, true), matGlow(0xffe9c4, 0.8));
        flameCore.rotation.x = Math.PI;
        flameCore.position.y = -0.55;
        flameCore.name = "flameCore";
        flameCore.visible = false;
        wrap.add(flame, flameCore);
        resolve(wrap);
      },
      undefined,
      () => {
        // fallback: procedural interceptor with named flame
        const g = buildInterceptor();
        const flame = new THREE.Mesh(new THREE.ConeGeometry(0.26, 1.5, 20, 1, true), matGlow(0xffc46e, 0.85));
        flame.rotation.x = Math.PI;
        flame.position.y = -0.8;
        flame.name = "flame";
        flame.visible = false;
        const flameCore = new THREE.Mesh(new THREE.ConeGeometry(0.13, 1.0, 16, 1, true), matGlow(0xffffff, 0.9));
        flameCore.rotation.x = Math.PI;
        flameCore.position.y = -0.55;
        flameCore.name = "flameCore";
        flameCore.visible = false;
        g.add(flame, flameCore);
        resolve(g);
      }
    );
  });
}

/* ---------- Canvas textures ---------- */
export function cloudTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const x = c.getContext("2d");
  const g = x.createRadialGradient(64, 64, 8, 64, 64, 62);
  g.addColorStop(0, "rgba(180,195,215,0.55)");
  g.addColorStop(0.55, "rgba(140,158,180,0.22)");
  g.addColorStop(1, "rgba(120,140,165,0)");
  x.fillStyle = g;
  x.fillRect(0, 0, 128, 128);
  const t = new THREE.CanvasTexture(c);
  return t;
}

export function smokeTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const x = c.getContext("2d");
  const g = x.createRadialGradient(64, 64, 6, 64, 64, 62);
  g.addColorStop(0, "rgba(220,205,190,0.5)");
  g.addColorStop(0.5, "rgba(170,160,150,0.25)");
  g.addColorStop(1, "rgba(140,130,122,0)");
  x.fillStyle = g;
  x.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

export function glowTexture(inner = "rgba(255,196,110,0.9)", outer = "rgba(255,171,64,0)") {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const x = c.getContext("2d");
  const g = x.createRadialGradient(32, 32, 2, 32, 32, 30);
  g.addColorStop(0, inner);
  g.addColorStop(1, outer);
  x.fillStyle = g;
  x.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

/* Seeded pseudo-random for deterministic layouts */
export function mulberry(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function earthTextures() {
  const rand = mulberry(20261117);
  const w = 1024, h = 512;
  const c = document.createElement("canvas");
  c.width = w; c.height = h;
  const x = c.getContext("2d");
  x.fillStyle = "#081a30";
  x.fillRect(0, 0, w, h);
  // continents: clustered ellipses
  x.fillStyle = "#15301f";
  for (let k = 0; k < 11; k++) {
    const cx = rand() * w, cy = h * 0.18 + rand() * h * 0.64;
    for (let i = 0; i < 26; i++) {
      const ex = cx + (rand() - 0.5) * 190, ey = cy + (rand() - 0.5) * 110;
      const rw = 14 + rand() * 52, rh = 8 + rand() * 30;
      x.beginPath();
      x.ellipse((ex + w) % w, ey, rw, rh, rand() * Math.PI, 0, Math.PI * 2);
      x.fill();
    }
  }
  // polar caps (subtle)
  x.fillStyle = "#44525f";
  x.fillRect(0, 0, w, 7);
  x.fillRect(0, h - 6, w, 6);
  const map = new THREE.CanvasTexture(c);
  // city lights on land pixels
  const data = x.getImageData(0, 0, w, h).data;
  const c2 = document.createElement("canvas");
  c2.width = w; c2.height = h;
  const x2 = c2.getContext("2d");
  x2.fillStyle = "#000";
  x2.fillRect(0, 0, w, h);
  x2.fillStyle = "#ffb050";
  let placed = 0, guard = 0;
  while (placed < 900 && guard++ < 20000) {
    const px = (rand() * w) | 0, py = (rand() * h) | 0;
    const i = (py * w + px) * 4;
    if (data[i] < 40 && data[i + 1] > 30) { // land color
      x2.globalAlpha = 0.35 + rand() * 0.65;
      x2.fillRect(px, py, rand() > 0.85 ? 2 : 1, 1);
      placed++;
    }
  }
  const lights = new THREE.CanvasTexture(c2);
  return { map, lights };
}
