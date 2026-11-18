/* ============================================================
   Mission Film — scroll-scrubbed Three.js world
   Deterministic: almost everything derives from scroll p.
   ============================================================ */
import * as THREE from "three";
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js";
import { RenderPass } from "three/addons/postprocessing/RenderPass.js";
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js";
import { ShaderPass } from "three/addons/postprocessing/ShaderPass.js";
import { OutputPass } from "three/addons/postprocessing/OutputPass.js";
import * as B from "./builders.js";

const V3 = (x, y, z) => new THREE.Vector3(x, y, z);
const lerp = (a, b, k) => a + (b - a) * k;
const clamp01 = (x) => Math.min(1, Math.max(0, x));
const smooth = (x) => x * x * (3 - 2 * x);
const vLerp = (out, a, b, k) => out.set(lerp(a.x, b.x, k), lerp(a.y, b.y, k), lerp(a.z, b.z, k));
const gauss = (x, c, w) => Math.exp(-((x - c) * (x - c)) / (w * w));

const GRADE = {
  uniforms: {
    tDiffuse: { value: null },
    uTint: { value: new THREE.Vector3(1, 1, 1) },
    uFlash: { value: 0 },
    uGrain: { value: 0.045 },
    uTime: { value: 0 },
  },
  vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }`,
  fragmentShader: `
    uniform sampler2D tDiffuse; uniform vec3 uTint; uniform float uFlash, uGrain, uTime;
    varying vec2 vUv;
    void main(){
      vec4 c = texture2D(tDiffuse, vUv);
      c.rgb *= uTint;
      c.rgb += uFlash;
      float d = distance(vUv, vec2(0.5));
      c.rgb *= 0.62 + 0.38 * smoothstep(0.85, 0.30, d);
      float g = fract(sin(dot(vUv + fract(uTime), vec2(12.9898, 78.233))) * 43758.5453);
      c.rgb += (g - 0.5) * uGrain;
      gl_FragColor = c;
    }`,
};

const SKY = {
  uniforms: { uTop: { value: new THREE.Color(0x01030a) }, uBottom: { value: new THREE.Color(0x0a1626) } },
  vertexShader: `varying float vY; void main(){ vY = normalize(position).y; gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }`,
  fragmentShader: `uniform vec3 uTop,uBottom; varying float vY;
    void main(){ gl_FragColor = vec4(mix(uBottom, uTop, clamp(vY*1.4+0.35, 0.0, 1.0)), 1.0); }`,
};

const ATMO = {
  uniforms: { uColor: { value: new THREE.Color(0x3d7fd9) } },
  vertexShader: `varying vec3 vN, vE; void main(){ vN = normalize(normalMatrix*normal); vec4 mv = modelViewMatrix*vec4(position,1.0); vE = normalize(-mv.xyz); gl_Position = projectionMatrix*mv; }`,
  fragmentShader: `uniform vec3 uColor; varying vec3 vN, vE;
    void main(){ float f = pow(1.0 - abs(dot(vN, vE)), 2.6); gl_FragColor = vec4(uColor, f*0.85); }`,
};

export async function createWorld(canvas, { tier = "hi" } = {}) {
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: tier === "hi", alpha: false });
  } catch (e) {
    return null;
  }
  const LOW = tier !== "hi";
  renderer.setPixelRatio(Math.min(devicePixelRatio, LOW ? 1.2 : 1.6));
  renderer.setSize(innerWidth, innerHeight);

  const scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x05070d, 0.016);
  const camera = new THREE.PerspectiveCamera(52, innerWidth / innerHeight, 0.1, 700);
  camera.position.set(16, 8, 22);
  const camPos = V3(16, 8, 22), camLook = V3(0, 3.5, 0);

  scene.add(new THREE.AmbientLight(0x404a58, 1.1));
  const key = new THREE.DirectionalLight(0xffd9a0, 1.35);
  key.position.set(6, 10, 6);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x5577cc, 0.5);
  rim.position.set(-8, 5, -7);
  scene.add(rim);

  /* ---------- composer ---------- */
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), LOW ? 0.6 : 0.85, 0.5, 0.42);
  composer.addPass(bloom);
  const grade = new ShaderPass(GRADE);
  composer.addPass(grade);
  composer.addPass(new OutputPass());

  /* ---------- sky + stars ---------- */
  const sky = new THREE.Mesh(new THREE.SphereGeometry(320, 18, 12), new THREE.ShaderMaterial({ ...SKY, uniforms: THREE.UniformsUtils.clone(SKY.uniforms), side: THREE.BackSide, depthWrite: false }));
  scene.add(sky);
  const skyU = sky.material.uniforms;

  const rand = B.mulberry(42);
  const N_ST = LOW ? 900 : 2300;
  const stPos = new Float32Array(N_ST * 3);
  for (let i = 0; i < N_ST; i++) {
    const r = 120 + rand() * 170, th = rand() * Math.PI * 2, ph = Math.acos(2 * rand() - 1);
    stPos[i * 3] = r * Math.sin(ph) * Math.cos(th);
    stPos[i * 3 + 1] = Math.abs(r * Math.cos(ph)) * 0.85 - 8;
    stPos[i * 3 + 2] = r * Math.sin(ph) * Math.sin(th);
  }
  const starGeo = new THREE.BufferGeometry();
  starGeo.setAttribute("position", new THREE.BufferAttribute(stPos, 3));
  const stars = new THREE.Points(starGeo, new THREE.PointsMaterial({ color: 0xcfd6de, size: 0.4, transparent: true, opacity: 0.6 }));
  scene.add(stars);

  /* ============ STAGE: PAD ============ */
  const pad = new THREE.Group();
  scene.add(pad);
  const ground = new THREE.Mesh(new THREE.CircleGeometry(180, 48), new THREE.MeshStandardMaterial({ color: 0x070b12, roughness: 0.92, metalness: 0.15 }));
  ground.rotation.x = -Math.PI / 2;
  pad.add(ground);
  for (const [r, op] of [[2.4, 0.35], [4.2, 0.2], [6.5, 0.12]]) {
    const ring = new THREE.Mesh(new THREE.RingGeometry(r - 0.05, r, 72), B.matGlow(B.AMBER, op));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    pad.add(ring);
  }
  const gantry = B.buildGantry();
  gantry.position.set(-2.5, 0, -0.7);
  pad.add(gantry);
  // floodlight towers + shafts
  const shaftMat = new THREE.MeshBasicMaterial({ color: 0xffc46e, transparent: true, opacity: 0.035, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide });
  for (const [tx, tz] of [[14, 14], [-14, 14], [14, -14], [-14, -14]]) {
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.1, 6.5), B.matHull(0x14171c));
    mast.position.set(tx, 3.25, tz);
    pad.add(mast);
    const lampGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: B.glowTexture(), transparent: true, opacity: 0.6, depthWrite: false }));
    lampGlow.position.set(tx, 6.6, tz);
    lampGlow.scale.setScalar(1.3);
    pad.add(lampGlow);
    // light shaft: apex up at the lamp, leaning gently toward the pad
    const shaft = new THREE.Mesh(new THREE.ConeGeometry(2.4, 7.5, 18, 1, true), shaftMat);
    shaft.position.set(tx * 0.88, 3.1, tz * 0.88);
    shaft.rotation.z = tx / 34;
    shaft.rotation.x = -tz / 34;
    pad.add(shaft);
  }
  // perimeter beacons
  const beacons = [];
  for (let i = 0; i < 10; i++) {
    const a = (i / 10) * Math.PI * 2;
    const b = new THREE.Mesh(new THREE.SphereGeometry(0.09, 8, 6), B.matGlow(0xff5050, 0.9));
    b.position.set(Math.cos(a) * 24, 0.35, Math.sin(a) * 24);
    pad.add(b);
    beacons.push(b);
  }
  // distant complex silhouettes
  for (let i = 0; i < 9; i++) {
    const w = 3 + rand() * 9, h = 2.5 + rand() * 9, d = 3 + rand() * 7;
    const bx = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), new THREE.MeshBasicMaterial({ color: 0x03050a }));
    const a = rand() * Math.PI * 2, r = 42 + rand() * 40;
    bx.position.set(Math.cos(a) * r, h / 2, Math.sin(a) * r);
    pad.add(bx);
  }
  // ground haze
  const smokeTex = B.smokeTexture();
  for (let i = 0; i < 3; i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: smokeTex, transparent: true, opacity: 0.05, blending: THREE.AdditiveBlending, depthWrite: false }));
    s.position.set(-6 + i * 6, 0.8, -3 + i * 2.5);
    s.scale.set(13, 3.8, 1);
    pad.add(s);
  }
  // ignition smoke sprites (p-driven)
  const smoke = [];
  for (let i = 0; i < (LOW ? 8 : 14); i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: smokeTex, color: 0x8f8880, transparent: true, opacity: 0, depthWrite: false }));
    const a = rand() * Math.PI * 2;
    s.userData = { dx: Math.cos(a), dz: Math.sin(a), r: 0.4 + rand(), v: 0.6 + rand() };
    pad.add(s);
    smoke.push(s);
  }
  const flash = new THREE.PointLight(0xffc080, 0, 50, 1.4);
  flash.position.set(0, 1.4, 0);
  pad.add(flash);
  // dawn sun
  const sun = new THREE.Sprite(new THREE.SpriteMaterial({ map: B.glowTexture("rgba(255,210,150,1)", "rgba(255,140,60,0)"), transparent: true, opacity: 0, depthWrite: false }));
  sun.position.set(-26, -3, -60);
  sun.scale.setScalar(55);
  pad.add(sun);
  // distant dawn launch streak
  const streakPts = [];
  for (let i = 0; i <= 50; i++) {
    const t = i / 50;
    streakPts.push(V3(-55 + t * 30, t * t * 46, -85));
  }
  const streak = new THREE.Line(new THREE.BufferGeometry().setFromPoints(streakPts), new THREE.LineBasicMaterial({ color: B.AMBER_HI, transparent: true, opacity: 0.8 }));
  streak.geometry.setDrawRange(0, 0);
  pad.add(streak);

  /* hero missile (GLB) */
  const heroA = await B.loadHero("../models/hero.glb");
  pad.add(heroA);
  const flameA = heroA.getObjectByName("flame");
  const flameCoreA = heroA.getObjectByName("flameCore");

  // exhaust particles under heroA
  const N_EX = LOW ? 120 : 260;
  const exPos = new Float32Array(N_EX * 3);
  const exGeo = new THREE.BufferGeometry();
  exGeo.setAttribute("position", new THREE.BufferAttribute(exPos, 3));
  const exhaust = new THREE.Points(exGeo, new THREE.PointsMaterial({ color: 0xffb060, size: 0.09, transparent: true, opacity: 0.7 }));
  exhaust.visible = false;
  pad.add(exhaust);

  /* clouds (launch fly-through) */
  const cloudTex = B.cloudTexture();
  const clouds = [];
  for (let i = 0; i < (LOW ? 5 : 11); i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: cloudTex, transparent: true, opacity: 0.0, depthWrite: false }));
    const a = rand() * Math.PI * 2, r = 3.5 + rand() * 7;
    s.position.set(Math.cos(a) * r, 9 + rand() * 13, Math.sin(a) * r);
    s.scale.setScalar(6 + rand() * 8);
    s.userData.base = 0.16 + rand() * 0.1;
    pad.add(s);
    clouds.push(s);
  }

  /* ============ STAGE: SKY (GNC + intercept) ============ */
  const skyStage = new THREE.Group();
  scene.add(skyStage);
  const heroB = heroA.clone(true);
  const heroBG = new THREE.Group();
  heroBG.add(heroB);
  heroB.position.y = -2.4; // center on origin
  skyStage.add(heroBG);
  const flameB = heroB.getObjectByName("flame");
  const flameCoreB = heroB.getObjectByName("flameCore");
  if (flameB) flameB.visible = true;
  if (flameCoreB) flameCoreB.visible = true;
  const engGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: B.glowTexture(), transparent: true, opacity: 0.5, depthWrite: false, blending: THREE.AdditiveBlending }));
  engGlow.scale.setScalar(1.15);
  skyStage.add(engGlow);

  const IMPACT = V3(0, 6, 0);
  const mCurve = new THREE.QuadraticBezierCurve3(V3(-42, -4, 10), V3(-12, 12, 2), IMPACT);
  const tStart = V3(48, 13, -28);
  const mPathPts = mCurve.getPoints(100);
  const mLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(mPathPts), new THREE.LineBasicMaterial({ color: B.AMBER_HI, transparent: true, opacity: 0.55 }));
  mLine.geometry.setDrawRange(0, 0);
  skyStage.add(mLine);
  const tPathPts = [];
  for (let i = 0; i <= 100; i++) tPathPts.push(V3().lerpVectors(tStart, IMPACT, i / 100));
  const tLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(tPathPts), new THREE.LineBasicMaterial({ color: 0xff5050, transparent: true, opacity: 0.5 }));
  tLine.geometry.setDrawRange(0, 0);
  skyStage.add(tLine);
  // lead line
  const leadGeo = new THREE.BufferGeometry().setFromPoints([V3(0, 0, 0), V3(0, 0, 0)]);
  const lead = new THREE.Line(leadGeo, new THREE.LineBasicMaterial({ color: B.AMBER, transparent: true, opacity: 0.22 }));
  skyStage.add(lead);
  // target drone
  const target = new THREE.Group();
  const tb = B.holo(new THREE.CylinderGeometry(0.22, 0.3, 2.2, 12), 0x2a1d1d);
  tb.rotation.z = Math.PI / 2;
  target.add(tb);
  const tn = B.holo(new THREE.ConeGeometry(0.22, 0.7, 12), 0x2a1d1d);
  tn.rotation.z = -Math.PI / 2;
  tn.position.x = -1.45;
  target.add(tn);
  const tGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: B.glowTexture("rgba(255,80,80,0.75)", "rgba(255,60,60,0)"), transparent: true, depthWrite: false }));
  tGlow.scale.setScalar(1.1);
  tGlow.position.x = 1.3;
  target.add(tGlow);
  skyStage.add(target);
  // explosion
  const N_BM = 260;
  const bmDir = [], bmSp = [];
  for (let i = 0; i < N_BM; i++) {
    const th = rand() * Math.PI * 2, ph = Math.acos(2 * rand() - 1);
    bmDir.push(V3(Math.sin(ph) * Math.cos(th), Math.cos(ph), Math.sin(ph) * Math.sin(th)));
    bmSp.push(0.35 + rand());
  }
  const bmPos = new Float32Array(N_BM * 3);
  const bmGeo = new THREE.BufferGeometry();
  bmGeo.setAttribute("position", new THREE.BufferAttribute(bmPos, 3));
  const boom = new THREE.Points(bmGeo, new THREE.PointsMaterial({ color: 0xffc46e, size: 0.16, transparent: true, opacity: 0 }));
  skyStage.add(boom);
  const shock = new THREE.Mesh(new THREE.RingGeometry(0.92, 1, 64), B.matGlow(0xffe0b0, 0.8));
  shock.position.copy(IMPACT);
  shock.visible = false;
  skyStage.add(shock);
  const boomFlash = new THREE.PointLight(0xffd0a0, 0, 60, 1.2);
  boomFlash.position.copy(IMPACT);
  skyStage.add(boomFlash);
  // high cloud deck far below
  for (let i = 0; i < 6; i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: cloudTex, transparent: true, opacity: 0.12, depthWrite: false }));
    s.position.set(-30 + i * 13, -16, -8 + (i % 2) * -14);
    s.scale.setScalar(26);
    skyStage.add(s);
  }

  /* ============ STAGE: ORBIT ============ */
  const orbitStage = new THREE.Group();
  scene.add(orbitStage);
  const { map: earthMap, lights: earthLights } = B.earthTextures();
  earthMap.colorSpace = THREE.SRGBColorSpace;
  const earth = new THREE.Mesh(
    new THREE.SphereGeometry(26, 48, 32),
    new THREE.MeshStandardMaterial({ map: earthMap, emissiveMap: earthLights, emissive: 0xffb050, emissiveIntensity: 0.85, roughness: 0.95, metalness: 0 })
  );
  earth.position.set(0, -30, -10);
  orbitStage.add(earth);
  const atmo = new THREE.Mesh(new THREE.SphereGeometry(26.9, 48, 32), new THREE.ShaderMaterial({ ...ATMO, uniforms: THREE.UniformsUtils.clone(ATMO.uniforms), transparent: true, blending: THREE.AdditiveBlending, side: THREE.BackSide, depthWrite: false }));
  atmo.position.copy(earth.position);
  orbitStage.add(atmo);
  const sat = B.buildSatellite();
  sat.scale.setScalar(1.3);
  sat.position.set(-1.6, 1.6, 3);
  sat.traverse((o) => {
    if (o.isMesh && o.material && o.material.emissive) o.material = o.material.clone();
  });
  sat.traverse((o) => {
    if (o.isMesh && o.material.emissiveIntensity !== undefined && o.material.emissive && o.material.emissive.getHex() === 0x0a1a2e)
      o.material.emissiveIntensity = 1.3;
  });
  orbitStage.add(sat);
  const orbEll = new THREE.EllipseCurve(0, 0, 30, 29, 0, Math.PI * 2);
  const orbPts = orbEll.getPoints(140).map((p2) => V3(p2.x, 0, p2.y));
  const orb = new THREE.Line(new THREE.BufferGeometry().setFromPoints(orbPts), new THREE.LineBasicMaterial({ color: B.AMBER, transparent: true, opacity: 0.3 }));
  orb.position.copy(earth.position);
  orb.rotation.set(0.42, 0, 0.12);
  orbitStage.add(orb);

  /* ============ STAGE: STATION ============ */
  const station = new THREE.Group();
  scene.add(station);
  const sGround = new THREE.Mesh(new THREE.PlaneGeometry(140, 140), new THREE.MeshStandardMaterial({ color: 0x080c12, roughness: 0.95 }));
  sGround.rotation.x = -Math.PI / 2;
  station.add(sGround);
  const sGrid = new THREE.GridHelper(70, 35, B.AMBER, 0x1c1a14);
  sGrid.material.transparent = true;
  sGrid.material.opacity = 0.14;
  station.add(sGrid);
  const dish = B.buildDish();
  dish.scale.setScalar(1.35);
  dish.position.set(-3.6, 0, -3);
  station.add(dish);
  // ops cabin
  const cabin = B.holo(new THREE.BoxGeometry(4.2, 1.7, 2.4), 0x14181e);
  cabin.position.set(3.6, 0.85, -1.2);
  station.add(cabin);
  for (let i = 0; i < 4; i++) {
    const win = new THREE.Mesh(new THREE.PlaneGeometry(0.55, 0.3), B.matGlow(0xffc46e, 0.75));
    win.position.set(2.4 + i * 0.8, 1.05, 0.01);
    station.add(win);
  }
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.06, 5.2), B.matHull(0x171a20));
  mast.position.set(6.4, 2.6, -3.4);
  station.add(mast);
  const mastLight = new THREE.Mesh(new THREE.SphereGeometry(0.09, 8, 6), B.matGlow(0xff5050));
  mastLight.position.set(6.4, 5.3, -3.4);
  station.add(mastLight);
  // uplink packets
  const upA = V3(-3.6, 4.9, -3.6), upB = V3(4.5, 46, -22);
  const N_PK = 46;
  const pkPos = new Float32Array(N_PK * 3);
  const pkGeo = new THREE.BufferGeometry();
  pkGeo.setAttribute("position", new THREE.BufferAttribute(pkPos, 3));
  const packets = new THREE.Points(pkGeo, new THREE.PointsMaterial({ color: B.AMBER_HI, size: 0.16, transparent: true, opacity: 0.85 }));
  station.add(packets);
  const beam = new THREE.Line(new THREE.BufferGeometry().setFromPoints([upA, upB]), new THREE.LineBasicMaterial({ color: B.AMBER, transparent: true, opacity: 0.12 }));
  station.add(beam);

  /* ============ STAGE: HANGAR ============ */
  const hangar = new THREE.Group();
  scene.add(hangar);
  const hFloor = new THREE.Mesh(new THREE.PlaneGeometry(70, 26), new THREE.MeshStandardMaterial({ color: 0x0a0d12, roughness: 0.85, metalness: 0.3 }));
  hFloor.rotation.x = -Math.PI / 2;
  hangar.add(hFloor);
  const hGrid = new THREE.GridHelper(70, 35, B.AMBER, 0x191712);
  hGrid.material.transparent = true;
  hGrid.material.opacity = 0.16;
  hangar.add(hGrid);
  const back = new THREE.Mesh(new THREE.PlaneGeometry(70, 12), new THREE.MeshStandardMaterial({ color: 0x0b0e13, roughness: 0.9 }));
  back.position.set(0, 6, -7);
  hangar.add(back);
  const heroC = heroA.clone(true);
  heroC.scale.setScalar(0.85);
  const bayModels = [heroC, B.buildLauncher(), B.buildSatellite(), B.buildUGV()];
  const satHangar = bayModels[2];
  satHangar.userData.panelPivots.forEach((pv) => (pv.rotation.z = 0));
  satHangar.position.y = 1.6;
  const bays = [];
  bayModels.forEach((m, i) => {
    const bay = new THREE.Group();
    const bx = -9.6 + i * 6.4;
    bay.position.x = bx;
    bay.add(m);
    hangar.add(bay);
    bays.push(bay);
    const ring = new THREE.Mesh(new THREE.RingGeometry(2.0, 2.08, 48), B.matGlow(B.AMBER, 0.3));
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(bx, 0.02, 0);
    hangar.add(ring);
    const shaft = new THREE.Mesh(new THREE.ConeGeometry(2.3, 8.5, 18, 1, true), shaftMat.clone());
    shaft.material.opacity = 0.06;
    shaft.position.set(bx, 4.25, 0);
    hangar.add(shaft);
    const lamp = new THREE.PointLight(0xffc080, 26, 11, 1.6);
    lamp.position.set(bx, 6.4, 1);
    hangar.add(lamp);
  });

  /* ---------- acts ---------- */
  const stages = { pad, sky: skyStage, orbit: orbitStage, station, hangar };
  const skyCol = {
    night: [0x01030a, 0x0a1626],
    space: [0x000104, 0x010309],
    orbit: [0x000103, 0x000206],
    hangarC: [0x020304, 0x060809],
    dawnA: [0x12233f, 0xc2642f],
  };
  const tints = {
    cool: new THREE.Vector3(0.94, 0.97, 1.08),
    space: new THREE.Vector3(0.92, 0.96, 1.1),
    flat: new THREE.Vector3(1, 1, 1),
    hang: new THREE.Vector3(1.02, 0.99, 0.94),
    dawn: new THREE.Vector3(1.1, 1.0, 0.9),
  };
  const topC = new THREE.Color(), botC = new THREE.Color();
  const tgtTop = new THREE.Color(0x01030a), tgtBot = new THREE.Color(0x0a1626);
  const tgtTint = tints.cool.clone();
  let fogTarget = 0.016, starsTarget = 0.55;

  const Q0 = 0.06; // ignition point within launch act
  const IMP = 0.7; // impact point within intercept act
  const missileAlt = (q) => (q <= Q0 ? 0 : 34 * Math.pow((q - Q0) / (1 - Q0), 1.7));

  const camA = V3(), camB = V3(), lookA = V3(), lookB = V3();
  const tmp = V3(), tmp2 = V3(), tan = V3();
  const qTmp = new THREE.Quaternion(), qRoll = new THREE.Quaternion();
  const YUP = V3(0, 1, 0);

  const ACTS = {
    title: {
      stage: "pad",
      run(q, t) {
        vLerp(camPos, camA.set(17, 9.5, 23), camB.set(8, 3.8, 13.5), smooth(q));
        vLerp(camLook, lookA.set(0, 4, 0), lookB.set(0, 2.8, 0), smooth(q));
        heroA.position.y = 0;
        setFlame(false, t);
        tgtTop.setHex(skyCol.night[0]); tgtBot.setHex(skyCol.night[1]);
        tgtTint.copy(tints.cool);
        fogTarget = 0.016; starsTarget = 0.5;
        sun.material.opacity = 0;
        cloudsOpacity(0);
      },
    },
    countdown: {
      stage: "pad",
      run(q, t) {
        vLerp(camPos, camA.set(1.35, 5.2, 3.3), camB.set(2.2, 0.75, 4.6), smooth(q));
        vLerp(camLook, lookA.set(0, 4.6, 0), lookB.set(0, 1.0, 0), smooth(q));
        heroA.position.y = 0;
        setFlame(false, t);
        tgtTop.setHex(skyCol.night[0]); tgtBot.setHex(skyCol.night[1]);
        tgtTint.copy(tints.cool);
        fogTarget = 0.016; starsTarget = 0.5;
        cloudsOpacity(0);
      },
    },
    launch: {
      stage: "pad",
      run(q, t) {
        const y = missileAlt(q);
        heroA.position.y = y;
        const lit = q >= Q0 * 0.82;
        setFlame(lit, t);
        if (q < Q0) {
          const k = smooth(q / Q0);
          vLerp(camPos, camA.set(2.2, 0.75, 4.6), camB.set(3.1, 1.3, 5.6), k);
          vLerp(camLook, lookA.set(0, 1.0, 0), lookB.set(0, 2.3, 0), k);
        } else {
          camPos.set(3.1 + q * 1.4, y * 0.965 + 1.7, 5.6 - q * 1.2);
          camLook.set(0, y + 1.5, 0);
        }
        // ignition fx (deterministic in q)
        flash.intensity = 46 * gauss(q, Q0, 0.035) + (lit ? 7 + Math.sin(t * 31) * 2.4 : 0);
        grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.5 * gauss(q, Q0, 0.022));
        world.shake = Math.max(world.shake, 0.5 * gauss(q, Q0, 0.05) + (lit && q < 0.5 ? 0.06 : 0));
        const sq = clamp01((q - Q0) / 0.42);
        for (const s of smoke) {
          const u = s.userData;
          const k = clamp01(sq * (0.6 + u.v * 0.7));
          s.position.set(u.dx * (0.7 + k * 3.4) * u.r, 0.4 + k * (1.0 + u.v * 2.0), u.dz * (0.7 + k * 3.4) * u.r);
          s.scale.setScalar((1.2 + u.r * 1.7) * (0.35 + k * 1.8));
          s.material.opacity = q > Q0 ? Math.pow(1 - k, 1.6) * 0.3 : 0;
        }
        cloudsOpacity(q > 0.25 ? 1 : clamp01((q - 0.1) / 0.15));
        tgtTop.setHex(skyCol.space[0]); tgtBot.setHex(skyCol.space[1]);
        tgtTint.copy(tints.cool);
        fogTarget = lerp(0.016, 0.0035, clamp01(q * 1.4));
        starsTarget = lerp(0.3, 1, clamp01((q - 0.35) / 0.5));
      },
    },
    gnc: {
      stage: "sky",
      run(q, t) {
        const s = clamp01(q) * 0.55;
        flyMissile(s, t);
        drawPursuit(s);
        vLerp(camPos, tmp2.copy(heroBG.position).add(camA.set(-7, 2.4, 8)), tmp2, 0);
        camPos.copy(heroBG.position).add(camA.set(-7, 2.4 + q * 1.2, 8 - q * 1.5));
        camLook.copy(heroBG.position).add(tan.clone().multiplyScalar(9));
        boomReset();
        tgtTop.setHex(skyCol.space[0]); tgtBot.setHex(skyCol.space[1]);
        tgtTint.copy(tints.space);
        fogTarget = 0.002; starsTarget = 1;
      },
    },
    intercept: {
      stage: "sky",
      run(q, t) {
        const s = q < IMP ? 0.55 + (q / IMP) * 0.45 : 1;
        flyMissile(Math.min(s, 0.999), t);
        drawPursuit(s);
        const boomQ = clamp01((q - IMP) / (1 - IMP));
        heroBG.visible = q < IMP;
        engGlow.visible = q < IMP;
        target.visible = q < IMP;
        if (q < IMP) {
          camPos.copy(heroBG.position).add(camA.set(-4, 1.6, 4.6));
          vLerp(camLook, tmp.copy(heroBG.position), IMPACT, smooth(clamp01(q / IMP)));
        } else {
          const a = 0.5 + boomQ * 0.9;
          camPos.set(IMPACT.x + Math.cos(a) * 8.5, IMPACT.y + 1.6 + boomQ * 1.2, IMPACT.z + Math.sin(a) * 8.5);
          camLook.copy(IMPACT);
        }
        // explosion (deterministic)
        if (boomQ > 0) {
          const e = Math.pow(boomQ, 0.55);
          for (let i = 0; i < N_BM; i++) {
            const d = bmDir[i], sp = bmSp[i];
            bmPos[i * 3] = IMPACT.x + d.x * e * 8.5 * sp;
            bmPos[i * 3 + 1] = IMPACT.y + d.y * e * 8.5 * sp - boomQ * boomQ * 2.2;
            bmPos[i * 3 + 2] = IMPACT.z + d.z * e * 8.5 * sp;
          }
          bmGeo.attributes.position.needsUpdate = true;
          boom.material.opacity = (1 - boomQ) * 0.95;
          shock.visible = true;
          shock.scale.setScalar(0.4 + Math.pow(boomQ, 0.5) * 17);
          shock.material.opacity = (1 - boomQ) * 0.75;
          shock.lookAt(camPos);
          boomFlash.intensity = 70 * gauss(boomQ, 0.05, 0.1);
          grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.7 * gauss(q, IMP + 0.015, 0.03));
          world.shake = Math.max(world.shake, 0.45 * gauss(q, IMP + 0.03, 0.08));
        } else boomReset();
        tgtTop.setHex(skyCol.space[0]); tgtBot.setHex(skyCol.space[1]);
        tgtTint.copy(tints.space);
        fogTarget = 0.002; starsTarget = 1;
      },
    },
    orbit: {
      stage: "orbit",
      run(q, t) {
        const k = smooth(q);
        vLerp(camPos, camA.set(8, 2.2, 15), camB.set(-5.5, 4.8, 11), k);
        vLerp(camLook, lookA.set(0, -3.5, -6), lookB.set(-0.5, 1.2, 2), k);
        earth.rotation.y = t * 0.012 + q * 0.5;
        const unfold = clamp01((q - 0.16) / 0.3);
        sat.userData.panelPivots.forEach((pv) => (pv.rotation.z = pv.userData.side * 1.5 * (1 - smooth(unfold))));
        sat.rotation.y = 0.4 + t * 0.04 + q * 1.6;
        sat.position.y = 1.4 + Math.sin(t * 0.4) * 0.12;
        tgtTop.setHex(skyCol.orbit[0]); tgtBot.setHex(skyCol.orbit[1]);
        tgtTint.copy(tints.flat);
        fogTarget = 0.0006; starsTarget = 1;
      },
    },
    station: {
      stage: "station",
      run(q, t) {
        const k = smooth(q);
        vLerp(camPos, camA.set(9.5, 2.8, 12.5), camB.set(2.2, 2.0, 8.2), k);
        vLerp(camLook, lookA.set(-2, 2.6, -2.4), lookB.set(-2.8, 3.2, -3), k);
        dish.userData.yoke.rotation.y = t * 0.14;
        dish.userData.bowl.rotation.x = -0.9 + Math.sin(t * 0.3) * 0.08;
        for (let i = 0; i < N_PK; i++) {
          const kk = (i / N_PK + t * 0.1) % 1;
          pkPos[i * 3] = lerp(upA.x, upB.x, kk);
          pkPos[i * 3 + 1] = lerp(upA.y, upB.y, kk);
          pkPos[i * 3 + 2] = lerp(upA.z, upB.z, kk);
        }
        pkGeo.attributes.position.needsUpdate = true;
        mastLight.material.opacity = 0.4 + 0.6 * (Math.sin(t * 2.6) > 0 ? 1 : 0.1);
        tgtTop.setHex(skyCol.night[0]); tgtBot.setHex(skyCol.night[1]);
        tgtTint.copy(tints.cool);
        fogTarget = 0.012; starsTarget = 0.8;
      },
    },
    hangar: {
      stage: "hangar",
      run(q, t) {
        const dolly = lerp(-11.5, 11.5, smooth(Math.min(q / 0.86, 1)));
        if (q < 0.86) {
          camPos.set(dolly, 2.4, 8.4);
          camLook.set(dolly * 0.94, 1.7, 0);
        } else {
          const b = smooth((q - 0.86) / 0.14);
          vLerp(camPos, camA.set(11.5, 2.4, 8.4), camB.set(0, 3.4, 13.5), b);
          vLerp(camLook, lookA.set(10.8, 1.7, 0), lookB.set(0, 1.7, 0), b);
        }
        bays.forEach((bay, i) => (bay.rotation.y = t * 0.12 + world.hangarYaw + i * 0.4));
        tgtTop.setHex(skyCol.hangarC[0]); tgtBot.setHex(skyCol.hangarC[1]);
        tgtTint.copy(tints.hang);
        fogTarget = 0.024; starsTarget = 0;
      },
    },
    dawn: {
      stage: "pad",
      run(q, t) {
        const k = smooth(q);
        vLerp(camPos, camA.set(15, 4.8, 21), camB.set(9.5, 3.2, 15.5), k);
        vLerp(camLook, lookA.set(0, 3.2, 0), lookB.set(-2, 3.6, -4), k);
        heroA.position.y = 0;
        setFlame(false, t);
        sun.material.opacity = clamp01(q * 1.4) * 0.95;
        sun.position.y = -4 + q * 9;
        const st = clamp01((q - 0.32) / 0.42);
        streak.geometry.setDrawRange(0, Math.floor(st * 50));
        streak.material.opacity = st >= 1 ? clamp01(1 - (q - 0.74) / 0.2) * 0.8 : 0.8;
        topC.setHex(skyCol.night[0]).lerp(new THREE.Color(skyCol.dawnA[0]), k);
        botC.setHex(skyCol.night[1]).lerp(new THREE.Color(skyCol.dawnA[1]), k * 0.85);
        tgtTop.copy(topC); tgtBot.copy(botC);
        tgtTint.copy(tints.cool).lerp(tints.dawn, k);
        fogTarget = 0.01; starsTarget = lerp(0.5, 0.05, k);
        cloudsOpacity(0);
      },
    },
  };

  function setFlame(on, t) {
    if (flameA) {
      flameA.visible = on;
      flameCoreA.visible = on;
      if (on) {
        flameA.scale.y = 1 + Math.sin(t * 30) * 0.18;
        flameCoreA.scale.y = 1 + Math.sin(t * 41) * 0.25;
      }
    }
    exhaust.visible = on;
  }
  function cloudsOpacity(k) {
    for (const s of clouds) s.material.opacity = s.userData.base * k;
  }
  function flyMissile(s, t) {
    mCurve.getPoint(s, heroBG.position);
    mCurve.getTangent(s, tan);
    qTmp.setFromUnitVectors(YUP, tan);
    qRoll.setFromAxisAngle(tan, Math.sin(s * Math.PI) * 0.7);
    heroBG.quaternion.copy(qRoll.multiply(qTmp));
    if (flameB) {
      flameB.scale.y = 1 + Math.sin(t * 34) * 0.2;
      flameCoreB.scale.y = 1 + Math.sin(t * 47) * 0.3;
    }
    engGlow.position.copy(heroBG.position).addScaledVector(tan, -2.5);
    target.position.lerpVectors(tStart, IMPACT, s);
    tmp.subVectors(IMPACT, tStart).normalize();
    target.lookAt(tmp2.copy(target.position).add(tmp));
    target.rotateY(Math.PI / 2);
  }
  function drawPursuit(s) {
    const n = Math.floor(clamp01(s) * 100);
    mLine.geometry.setDrawRange(0, n);
    tLine.geometry.setDrawRange(0, n);
    const lp = lead.geometry.attributes.position.array;
    lp[0] = heroBG.position.x; lp[1] = heroBG.position.y; lp[2] = heroBG.position.z;
    lp[3] = IMPACT.x; lp[4] = IMPACT.y; lp[5] = IMPACT.z;
    lead.geometry.attributes.position.needsUpdate = true;
    lead.visible = s < 1;
  }
  function boomReset() {
    boom.material.opacity = 0;
    shock.visible = false;
    boomFlash.intensity = 0;
    heroBG.visible = true;
    engGlow.visible = true;
    target.visible = true;
  }

  /* ---------- act ranges & frame loop ---------- */
  let ranges = [];
  const ids = ["title", "countdown", "launch", "gnc", "intercept", "orbit", "station", "hangar", "dawn"];
  ids.forEach((id, i) => ranges.push({ id, start: i / 9, end: (i + 1) / 9 }));

  const world = {
    renderer,
    shake: 0,
    hangarYaw: 0,
    pointer: { x: 0, y: 0 },
    currentAct: "title",
    setActRanges(r) { ranges = r; },
    resize() {
      camera.aspect = innerWidth / innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(innerWidth, innerHeight);
      composer.setSize(innerWidth, innerHeight);
    },
    update(p, dt, t) {
      let act = ranges[0], q = 0;
      for (const r of ranges) {
        if (p >= r.start || r === ranges[0]) { act = r; }
      }
      for (let i = ranges.length - 1; i >= 0; i--) {
        if (p >= ranges[i].start) { act = ranges[i]; break; }
      }
      q = clamp01((p - act.start) / Math.max(act.end - act.start, 1e-4));
      world.currentAct = act.id;
      const def = ACTS[act.id] || ACTS.title;
      for (const [name, g] of Object.entries(stages)) g.visible = name === def.stage;

      world.shake = Math.max(0, world.shake - dt * 0.9);
      grade.uniforms.uFlash.value = Math.max(0, grade.uniforms.uFlash.value - dt * 2.4);
      def.run(q, t);

      // exhaust particles under heroA
      if (exhaust.visible) {
        const base = heroA.position.y - 0.4;
        for (let i = 0; i < N_EX; i++) {
          const l = ((t * (0.9 + (i % 7) * 0.08)) + i * 0.137) % 1;
          const ang = i * 2.399;
          exPos[i * 3] = Math.cos(ang) * 0.2 * (1 + l * 2.4);
          exPos[i * 3 + 1] = base - l * 4.6;
          exPos[i * 3 + 2] = Math.sin(ang) * 0.2 * (1 + l * 2.4);
        }
        exGeo.attributes.position.needsUpdate = true;
      }
      // beacons blink
      beacons.forEach((b, i) => (b.material.opacity = Math.sin(t * 2.2 + i * 1.7) > 0.2 ? 0.95 : 0.12));

      // smooth environment lerps
      const el = 1 - Math.pow(0.0015, dt);
      skyU.uTop.value.lerp(tgtTop, el);
      skyU.uBottom.value.lerp(tgtBot, el);
      grade.uniforms.uTint.value.lerp(tgtTint, el);
      scene.fog.density = lerp(scene.fog.density, fogTarget, el);
      stars.material.opacity = lerp(stars.material.opacity, starsTarget, el);
      grade.uniforms.uTime.value = t % 10;
      stars.rotation.y = t * 0.003;

      // target screen position for the seeker reticle
      if ((act.id === "gnc" || act.id === "intercept") && target.visible) {
        tmp.setFromMatrixPosition(target.matrixWorld).project(camera);
        world.targetScreen = {
          x: (tmp.x * 0.5 + 0.5) * innerWidth,
          y: (-tmp.y * 0.5 + 0.5) * innerHeight,
          on: tmp.z < 1 && Math.abs(tmp.x) < 1.1 && Math.abs(tmp.y) < 1.1,
        };
      } else world.targetScreen = { on: false };

      // camera: base + parallax + shake
      const shx = (Math.sin(t * 39) * 0.6 + Math.sin(t * 57 + 1.3) * 0.4) * world.shake;
      const shy = (Math.sin(t * 47 + 2.1) * 0.6 + Math.sin(t * 31) * 0.4) * world.shake;
      camera.position.set(
        camPos.x + world.pointer.x * 0.45 + shx,
        camPos.y - world.pointer.y * 0.3 + shy,
        camPos.z
      );
      camera.lookAt(camLook);
      composer.render();
    },
  };
  return world;
}
