/* ============================================================
   THE RANGE — procedural PBR military vehicles
   Generic & illustrative. All face +X, grounded at y=0.
   ============================================================ */
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export function mulberry(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/* ---------- textures ---------- */
export function camoTexture(seed = 7, base = "#6b6248", blotch = "#4a4a38", dark = "#35322a") {
  const rand = mulberry(seed);
  const c = document.createElement("canvas");
  c.width = c.height = 512;
  const x = c.getContext("2d");
  x.fillStyle = base;
  x.fillRect(0, 0, 512, 512);
  for (const [col, n, rmin, rmax] of [[blotch, 26, 28, 90], [dark, 18, 18, 55]]) {
    x.fillStyle = col;
    for (let i = 0; i < n; i++) {
      x.beginPath();
      x.ellipse(rand() * 512, rand() * 512, rmin + rand() * (rmax - rmin), rmin * 0.6 + rand() * rmax * 0.5, rand() * Math.PI, 0, Math.PI * 2);
      x.fill();
    }
  }
  // dust / wear speckle
  for (let i = 0; i < 2600; i++) {
    const g = 60 + (rand() * 70) | 0;
    x.fillStyle = `rgba(${g + 40},${g + 30},${g},${0.05 + rand() * 0.1})`;
    x.fillRect(rand() * 512, rand() * 512, 1 + rand() * 2.5, 1 + rand() * 2.5);
  }
  // panel scratches
  x.strokeStyle = "rgba(30,28,22,0.25)";
  for (let i = 0; i < 40; i++) {
    x.beginPath();
    const sx = rand() * 512, sy = rand() * 512;
    x.moveTo(sx, sy);
    x.lineTo(sx + (rand() - 0.5) * 60, sy + (rand() - 0.5) * 60);
    x.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  return t;
}

export function tubesTexture() {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 192;
  const x = c.getContext("2d");
  x.fillStyle = "#4e4a3c";
  x.fillRect(0, 0, 256, 192);
  for (let r = 0; r < 3; r++)
    for (let col = 0; col < 4; col++) {
      const cx = 36 + col * 62, cy = 34 + r * 62;
      x.fillStyle = "#15130f";
      x.beginPath(); x.arc(cx, cy, 22, 0, Math.PI * 2); x.fill();
      x.strokeStyle = "#6a6454";
      x.lineWidth = 4;
      x.beginPath(); x.arc(cx, cy, 24, 0, Math.PI * 2); x.stroke();
    }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

/* ---------- materials ---------- */
export const mats = {};
export function initMats() {
  mats.howitzer = new THREE.MeshStandardMaterial({ map: camoTexture(11, "#6f6549", "#4c4c3a", "#38352c"), roughness: 0.72, metalness: 0.28 });
  mats.tank = new THREE.MeshStandardMaterial({ map: camoTexture(23, "#5d5f4a", "#434636", "#30332a"), roughness: 0.75, metalness: 0.25 });
  mats.mlrs = new THREE.MeshStandardMaterial({ map: camoTexture(37, "#6a6150", "#4b473b", "#36332b"), roughness: 0.74, metalness: 0.26 });
  mats.truck = new THREE.MeshStandardMaterial({ map: camoTexture(51, "#63604c", "#46463a", "#33312a"), roughness: 0.7, metalness: 0.3 });
  mats.metal = new THREE.MeshStandardMaterial({ color: 0x2a2c2e, roughness: 0.45, metalness: 0.75 });
  mats.barrel = new THREE.MeshStandardMaterial({ color: 0x33352f, roughness: 0.5, metalness: 0.65 });
  mats.rubber = new THREE.MeshStandardMaterial({ color: 0x141517, roughness: 0.95, metalness: 0.05 });
  mats.track = new THREE.MeshStandardMaterial({ color: 0x1b1c1d, roughness: 0.9, metalness: 0.35 });
  mats.glass = new THREE.MeshStandardMaterial({ color: 0x0c1116, roughness: 0.12, metalness: 0.9 });
  mats.jet = new THREE.MeshStandardMaterial({ color: 0x5a6068, roughness: 0.4, metalness: 0.65 });
  mats.jetDark = new THREE.MeshStandardMaterial({ color: 0x2e3338, roughness: 0.5, metalness: 0.6 });
  // flatShading: the low-poly GLB's vertex normals are degenerate in places and
  // produce NaN in the env-map BRDF, which the bloom pass smears into black boxes.
  mats.missile = new THREE.MeshStandardMaterial({ color: 0x5f6450, roughness: 0.5, metalness: 0.45, flatShading: true });
  mats.tubes = new THREE.MeshStandardMaterial({ map: tubesTexture(), roughness: 0.7, metalness: 0.3 });
}

function M(geo, mat) {
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}
const box = (w, h, d, mat) => M(new THREE.BoxGeometry(w, h, d), mat);
const cyl = (r1, r2, h, mat, seg = 20) => M(new THREE.CylinderGeometry(r1, r2, h, seg), mat);

/* ---------- running gear ---------- */
function trackAssembly(len, side, mat = mats.track) {
  const g = new THREE.Group();
  const body = box(len, 0.62, 0.52, mat);
  body.position.y = 0.42;
  g.add(body);
  const n = Math.max(4, Math.round(len / 1.05));
  for (let i = 0; i < n; i++) {
    const w = cyl(0.3, 0.3, 0.58, mats.rubber, 16);
    w.rotation.x = Math.PI / 2;
    w.position.set(-len / 2 + 0.55 + i * ((len - 1.1) / (n - 1)), 0.32, 0);
    g.add(w);
  }
  g.position.z = side;
  return g;
}

function wheelAxle(x, z, r = 0.52) {
  const w = cyl(r, r, 0.45, mats.rubber, 18);
  w.rotation.x = Math.PI / 2;
  w.position.set(x, r, z);
  const hub = cyl(r * 0.45, r * 0.45, 0.47, mats.metal, 12);
  hub.rotation.x = Math.PI / 2;
  hub.position.copy(w.position);
  const g = new THREE.Group();
  g.add(w, hub);
  return g;
}

/* ---------- SELF-PROPELLED HOWITZER ---------- */
export function buildHowitzer() {
  const g = new THREE.Group();
  const hull = box(6.6, 1.15, 3.1, mats.howitzer);
  hull.position.y = 1.18;
  g.add(hull);
  const glacis = box(1.5, 1.05, 3.1, mats.howitzer);
  glacis.position.set(3.45, 1.05, 0);
  glacis.rotation.z = -0.42;
  g.add(glacis);
  g.add(trackAssembly(6.9, 1.62, mats.track));
  g.add(trackAssembly(6.9, -1.62, mats.track));
  const skirtL = box(7.0, 0.3, 0.12, mats.howitzer); skirtL.position.set(0, 0.95, 1.68); g.add(skirtL);
  const skirtR = skirtL.clone(); skirtR.position.z = -1.68; g.add(skirtR);
  // turret
  const turret = new THREE.Group();
  turret.position.set(-0.7, 1.78, 0);
  const tb = box(3.4, 1.25, 2.9, mats.howitzer);
  tb.position.y = 0.6;
  turret.add(tb);
  const tFront = box(1.0, 1.1, 2.9, mats.howitzer);
  tFront.position.set(2.0, 0.5, 0);
  tFront.rotation.z = -0.3;
  turret.add(tFront);
  const hatch = cyl(0.42, 0.42, 0.18, mats.metal, 16);
  hatch.position.set(-0.7, 1.3, 0.8);
  turret.add(hatch);
  const stow = box(0.5, 0.45, 2.5, mats.metal);
  stow.position.set(-1.8, 0.7, 0);
  turret.add(stow);
  // barrel on its own pivot
  const barrelPivot = new THREE.Group();
  barrelPivot.position.set(1.6, 0.85, 0);
  const recoil = new THREE.Group();
  const sleeve = cyl(0.26, 0.3, 1.7, mats.barrel, 18);
  sleeve.rotation.z = Math.PI / 2;
  sleeve.position.x = 1.0;
  recoil.add(sleeve);
  const tube = cyl(0.14, 0.18, 5.4, mats.barrel, 18);
  tube.rotation.z = Math.PI / 2;
  tube.position.x = 4.3;
  recoil.add(tube);
  const brake = cyl(0.24, 0.24, 0.72, mats.barrel, 18);
  brake.rotation.z = Math.PI / 2;
  brake.position.x = 7.15;
  recoil.add(brake);
  for (const dx of [-0.18, 0.12]) {
    const ring = M(new THREE.TorusGeometry(0.25, 0.035, 10, 20), mats.metal);
    ring.rotation.y = Math.PI / 2;
    ring.position.x = 7.15 + dx;
    recoil.add(ring);
  }
  barrelPivot.add(recoil);
  turret.add(barrelPivot);
  g.add(turret);
  g.userData = { turret, barrelPivot, recoil, muzzleLocal: new THREE.Vector3(7.5, 0, 0) };
  return g;
}

/* ---------- MAIN BATTLE TANK ---------- */
export function buildTank() {
  const g = new THREE.Group();
  const hull = box(6.2, 0.95, 3.0, mats.tank);
  hull.position.y = 1.05;
  g.add(hull);
  const glacis = box(1.7, 0.8, 3.0, mats.tank);
  glacis.position.set(3.15, 0.95, 0);
  glacis.rotation.z = -0.5;
  g.add(glacis);
  g.add(trackAssembly(6.4, 1.58));
  g.add(trackAssembly(6.4, -1.58));
  const skirtL = box(6.5, 0.42, 0.12, mats.tank); skirtL.position.set(0, 0.86, 1.64); g.add(skirtL);
  const skirtR = skirtL.clone(); skirtR.position.z = -1.64; g.add(skirtR);
  const turret = new THREE.Group();
  turret.position.set(0.25, 1.52, 0);
  const tb = box(2.7, 0.75, 2.3, mats.tank);
  tb.position.y = 0.38;
  turret.add(tb);
  const cheekL = box(1.2, 0.72, 0.65, mats.tank); cheekL.position.set(1.15, 0.36, 0.95); cheekL.rotation.y = 0.5; turret.add(cheekL);
  const cheekR = cheekL.clone(); cheekR.position.z = -0.95; cheekR.rotation.y = -0.5; turret.add(cheekR);
  const bustle = box(1.3, 0.6, 2.1, mats.metal); bustle.position.set(-1.6, 0.35, 0); turret.add(bustle);
  const sight = box(0.4, 0.3, 0.5, mats.metal); sight.position.set(0.3, 0.9, -0.6); turret.add(sight);
  const barrelPivot = new THREE.Group();
  barrelPivot.position.set(1.35, 0.35, 0);
  const tube = cyl(0.11, 0.14, 4.6, mats.barrel, 16);
  tube.rotation.z = Math.PI / 2;
  tube.position.x = 2.5;
  barrelPivot.add(tube);
  const sleeve = cyl(0.17, 0.17, 1.1, mats.barrel, 16);
  sleeve.rotation.z = Math.PI / 2;
  sleeve.position.x = 3.1;
  barrelPivot.add(sleeve);
  turret.add(barrelPivot);
  g.add(turret);
  g.userData = { turret, barrelPivot };
  return g;
}

/* ---------- MLRS ---------- */
export function buildMLRS() {
  const g = new THREE.Group();
  const hull = box(6.4, 1.0, 2.9, mats.mlrs);
  hull.position.y = 1.1;
  g.add(hull);
  g.add(trackAssembly(6.6, 1.55));
  g.add(trackAssembly(6.6, -1.55));
  const cab = box(1.9, 1.1, 2.9, mats.mlrs);
  cab.position.set(2.5, 2.1, 0);
  g.add(cab);
  const wind = box(0.06, 0.5, 2.5, mats.glass);
  wind.position.set(3.47, 2.25, 0);
  wind.rotation.z = -0.25;
  g.add(wind);
  // launch pod on pivot
  const pod = new THREE.Group();
  pod.position.set(-1.3, 1.85, 0);
  const podBox = box(4.4, 1.5, 2.6, mats.mlrs);
  podBox.position.x = -0.6;
  pod.add(podBox);
  const face = M(new THREE.PlaneGeometry(1.5, 2.55), mats.tubes);
  face.rotation.y = Math.PI / 2;
  face.rotation.z = Math.PI / 2;
  face.position.set(1.62, 0, 0);
  pod.add(face);
  const frame = box(0.3, 1.7, 2.8, mats.metal);
  frame.position.x = -2.9;
  pod.add(frame);
  pod.rotation.z = 0.5; // elevated
  g.add(pod);
  g.userData = { pod, muzzleLocal: new THREE.Vector3(1.8, 0, 0) };
  return g;
}

/* ---------- TEL (transporter-erector-launcher) ---------- */
export function buildTEL(missile) {
  const g = new THREE.Group();
  const chassis = box(9.2, 0.55, 2.6, mats.truck);
  chassis.position.y = 1.05;
  g.add(chassis);
  const cab = box(1.9, 1.5, 2.6, mats.truck);
  cab.position.set(3.9, 2.0, 0);
  g.add(cab);
  const wind = box(0.06, 0.55, 2.2, mats.glass);
  wind.position.set(4.87, 2.2, 0);
  wind.rotation.z = -0.18;
  g.add(wind);
  const bed = box(6.6, 0.25, 2.4, mats.metal);
  bed.position.set(-0.9, 1.42, 0);
  g.add(bed);
  for (const x of [3.3, 1.9, -1.4, -2.9, -4.4]) {
    g.add(wheelAxle(x, 1.38));
    g.add(wheelAxle(x, -1.38));
  }
  // erector
  const erector = new THREE.Group();
  erector.position.set(-4.0, 1.55, 0);
  const rail = box(7.0, 0.3, 1.0, mats.metal);
  rail.position.x = 3.4;
  erector.add(rail);
  const cradleA = box(0.25, 0.7, 1.5, mats.metal); cradleA.position.set(1.2, 0.4, 0); erector.add(cradleA);
  const cradleB = box(0.25, 0.7, 1.5, mats.metal); cradleB.position.set(5.2, 0.4, 0); erector.add(cradleB);
  if (missile) {
    missile.rotation.z = -Math.PI / 2; // lie along +x on the rail
    missile.position.set(0.6, 0.78, 0);
    erector.add(missile);
  }
  g.add(erector);
  g.userData = { erector, missile };
  return g;
}

/* ---------- FIGHTER JET ---------- */
export function buildJet() {
  const g = new THREE.Group();
  const pts = [];
  const prof = [[0, 0], [0.18, 0.5], [0.34, 1.6], [0.42, 3.2], [0.44, 6.2], [0.4, 8.2], [0.3, 9.4], [0.26, 10.0]];
  for (const [r, y] of prof) pts.push(new THREE.Vector2(r, y));
  const fus = M(new THREE.LatheGeometry(pts, 20), mats.jet);
  fus.rotation.z = -Math.PI / 2; // nose +x
  fus.position.x = -4.6;
  g.add(fus);
  // canopy
  const can = M(new THREE.SphereGeometry(0.34, 16, 12), mats.glass);
  can.scale.set(1.9, 0.75, 0.75);
  can.position.set(2.6, 0.42, 0);
  g.add(can);
  // delta wings
  const wShape = new THREE.Shape();
  wShape.moveTo(0, 0); wShape.lineTo(-3.4, 3.1); wShape.lineTo(-4.6, 3.1); wShape.lineTo(-2.4, 0); wShape.closePath();
  const wGeo = new THREE.ExtrudeGeometry(wShape, { depth: 0.1, bevelEnabled: false });
  for (const s of [1, -1]) {
    const w = M(wGeo, mats.jet);
    w.rotation.x = s === 1 ? Math.PI / 2 : -Math.PI / 2;
    w.position.set(1.6, 0.05, s * 0.3);
    g.add(w);
  }
  // tail fin
  const fShape = new THREE.Shape();
  fShape.moveTo(0, 0); fShape.lineTo(-1.3, 1.5); fShape.lineTo(-2.0, 1.5); fShape.lineTo(-1.1, 0); fShape.closePath();
  const fin = M(new THREE.ExtrudeGeometry(fShape, { depth: 0.08, bevelEnabled: false }), mats.jetDark);
  fin.position.set(-3.2, 0.3, -0.04);
  g.add(fin);
  // tailplanes
  for (const s of [1, -1]) {
    const tp = M(wGeo, mats.jetDark);
    tp.scale.setScalar(0.45);
    tp.rotation.x = s === 1 ? Math.PI / 2 : -Math.PI / 2;
    tp.position.set(-3.3, 0.05, s * 0.25);
    g.add(tp);
  }
  // nozzle
  const noz = cyl(0.3, 0.38, 0.7, mats.jetDark, 16);
  noz.rotation.z = Math.PI / 2;
  noz.position.set(-4.9, 0, 0);
  g.add(noz);
  g.userData = { nozzleLocal: new THREE.Vector3(-5.3, 0, 0) };
  return g;
}

/* ---------- missile (PBR, from hero.glb with military repaint) ---------- */
export function loadMissilePBR(url) {
  return new Promise((resolve) => {
    const finish = (wrap) => resolve(wrap);
    new GLTFLoader().load(
      url,
      (gltf) => {
        const m = gltf.scene;
        m.traverse((o) => {
          if (o.isMesh) {
            o.material = mats.missile;
            o.castShadow = o.receiveShadow = true;
          }
        });
        const bb = new THREE.Box3().setFromObject(m);
        const size = bb.getSize(new THREE.Vector3());
        const s = 5.6 / Math.max(size.x, size.y, size.z);
        const wrap = new THREE.Group();
        wrap.add(m);
        m.scale.setScalar(s);
        bb.setFromObject(m);
        const c = bb.getCenter(new THREE.Vector3());
        m.position.set(-c.x, -c.y, -c.z); // centered at origin
        finish(wrap);
      },
      undefined,
      () => {
        // procedural fallback
        const wrap = new THREE.Group();
        const body = cyl(0.26, 0.3, 4.2, mats.missile, 22);
        wrap.add(body);
        const nose = cyl(0.02, 0.26, 1.2, mats.missile, 22);
        nose.position.y = 2.7;
        wrap.add(nose);
        for (let i = 0; i < 4; i++) {
          const fin = box(0.7, 0.9, 0.05, mats.missile);
          fin.position.y = -1.7;
          fin.rotation.y = (i * Math.PI) / 2;
          fin.translateX(0.5);
          wrap.add(fin);
        }
        finish(wrap);
      }
    );
  });
}
