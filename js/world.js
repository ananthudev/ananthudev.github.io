/* ============================================================
   THE RANGE — golden-hour live-fire film (scroll-scrubbed)
   Deterministic: every event derives from scroll p.
   ============================================================ */
import * as THREE from "three";
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js";
import { RenderPass } from "three/addons/postprocessing/RenderPass.js";
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js";
import { ShaderPass } from "three/addons/postprocessing/ShaderPass.js";
import { OutputPass } from "three/addons/postprocessing/OutputPass.js";
import * as V from "./vehicles.js";

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
    uGrain: { value: 0.035 },
    uHaze: { value: 0.0 },
    uTime: { value: 0 },
  },
  vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }`,
  fragmentShader: `
    uniform sampler2D tDiffuse; uniform vec3 uTint; uniform float uFlash,uGrain,uHaze,uTime;
    varying vec2 vUv;
    void main(){
      vec2 uv = vUv;
      // heat shimmer: wobble a narrow band around the horizon
      float band = smoothstep(0.38, 0.47, uv.y) * (1.0 - smoothstep(0.50, 0.60, uv.y));
      uv.x += sin(uv.y*160.0 + uTime*9.0) * uHaze * band;
      uv.y += cos(uv.x*120.0 + uTime*7.0) * uHaze * 0.55 * band;
      vec4 c = texture2D(tDiffuse, uv);
      c.rgb *= uTint;
      c.rgb += uFlash;
      float d = distance(vUv, vec2(0.5));
      c.rgb *= 0.70 + 0.30 * smoothstep(0.86, 0.32, d);
      float g = fract(sin(dot(vUv + fract(uTime*0.73), vec2(12.9898,78.233))) * 43758.5453);
      c.rgb += (g - 0.5) * uGrain;
      gl_FragColor = c;
    }`,
};

const SKY = {
  uniforms: {
    uTop: { value: new THREE.Color(0x2a3550) },
    uHorizon: { value: new THREE.Color(0xd98a4a) },
    uSunDir: { value: new THREE.Vector3(0.9, 0.08, -0.3).normalize() },
    uSunGlow: { value: 1.0 },
  },
  vertexShader: `varying vec3 vDir; void main(){ vDir = normalize(position); gl_Position = projectionMatrix*modelViewMatrix*vec4(position,1.0); }`,
  fragmentShader: `
    uniform vec3 uTop,uHorizon,uSunDir; uniform float uSunGlow; varying vec3 vDir;
    void main(){
      float h = clamp(vDir.y*1.8 + 0.12, 0.0, 1.0);
      vec3 col = mix(uHorizon, uTop, pow(h, 0.72));
      float s = max(dot(normalize(vDir), uSunDir), 0.0);
      col += uHorizon * pow(s, 14.0) * 0.55 * uSunGlow;   // warm bleed around sun
      col += vec3(1.0, 0.86, 0.66) * pow(s, 180.0) * 2.2 * uSunGlow; // disc
      gl_FragColor = vec4(col, 1.0);
    }`,
};

/* terrain height — gentle dunes, flattened at vehicle pads and the road */
const PADS = [[0, 0, 15], [-30, 30, 12], [-54, -16, 13], [-16, 50, 13]];
function terrainH(x, z) {
  let h = Math.sin(x * 0.021) * Math.cos(z * 0.027) * 2.1
        + Math.sin(x * 0.055 + 1.7) * Math.sin(z * 0.043) * 0.9
        + Math.sin(x * 0.13 + z * 0.11) * 0.25;
  h = Math.max(h, -1.2);
  let flat = 1;
  for (const [px, pz, pr] of PADS) {
    const d = Math.hypot(x - px, z - pz);
    flat = Math.min(flat, smooth(clamp01((d - pr) / 14)));
  }
  // road corridor downrange (+x, |z| small)
  if (x > -20) flat = Math.min(flat, smooth(clamp01((Math.abs(z) - 5) / 12)));
  return h * flat;
}

function terrainTexture() {
  const rand = V.mulberry(90);
  const c = document.createElement("canvas");
  c.width = c.height = 1024;
  const x = c.getContext("2d");
  x.fillStyle = "#8a7354";
  x.fillRect(0, 0, 1024, 1024);
  for (let i = 0; i < 9000; i++) {
    const g = 95 + (rand() * 70) | 0;
    x.fillStyle = `rgba(${g + 30},${g + 8},${(g * 0.72) | 0},${0.12 + rand() * 0.2})`;
    const s = 1 + rand() * 5;
    x.fillRect(rand() * 1024, rand() * 1024, s, s);
  }
  // scrub patches
  x.fillStyle = "rgba(70,70,44,0.5)";
  for (let i = 0; i < 260; i++) {
    x.beginPath();
    x.ellipse(rand() * 1024, rand() * 1024, 2 + rand() * 7, 1.5 + rand() * 4, rand() * 3, 0, Math.PI * 2);
    x.fill();
  }
  // dirt road: world x∈[-250,250]→u, z→v; road along +x at z≈0 → v≈0.5
  const grd = x.createLinearGradient(0, 472, 0, 552);
  grd.addColorStop(0, "rgba(96,78,58,0)");
  grd.addColorStop(0.3, "rgba(96,78,58,0.85)");
  grd.addColorStop(0.7, "rgba(96,78,58,0.85)");
  grd.addColorStop(1, "rgba(96,78,58,0)");
  x.fillStyle = grd;
  x.fillRect(452, 472, 572, 80);
  x.strokeStyle = "rgba(58,48,38,0.7)";
  x.lineWidth = 5;
  for (const off of [-14, 14]) {
    x.beginPath();
    x.moveTo(452, 512 + off);
    x.lineTo(1024, 512 + off + 6);
    x.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function glowTex(inner = "rgba(255,220,170,1)", outer = "rgba(255,150,60,0)") {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const x = c.getContext("2d");
  const g = x.createRadialGradient(64, 64, 4, 64, 64, 62);
  g.addColorStop(0, inner);
  g.addColorStop(1, outer);
  x.fillStyle = g;
  x.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}
function smokeTex() {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const x = c.getContext("2d");
  const g = x.createRadialGradient(64, 64, 8, 64, 64, 62);
  g.addColorStop(0, "rgba(200,190,178,0.55)");
  g.addColorStop(0.55, "rgba(150,140,128,0.26)");
  g.addColorStop(1, "rgba(120,112,102,0)");
  x.fillStyle = g;
  x.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

export async function createWorld(canvas, { tier = "hi" } = {}) {
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: false });
  } catch (e) {
    return null;
  }
  const LOW = tier !== "hi";
  renderer.setPixelRatio(Math.min(devicePixelRatio, LOW ? 1.15 : 1.45));
  renderer.setSize(innerWidth, innerHeight);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  V.initMats();

  const scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x8a6b4e, 0.0055);
  const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 900);
  const camPos = V3(26, 3.4, 30), camLook = V3(0, 2, 0);
  camera.position.copy(camPos);

  /* ---------- lights ---------- */
  const sun = new THREE.DirectionalLight(0xffb46b, 2.6);
  sun.position.set(120, 16, -40);
  sun.castShadow = true;
  sun.shadow.mapSize.set(LOW ? 1024 : 1536, LOW ? 1024 : 1536);
  sun.shadow.camera.left = -45; sun.shadow.camera.right = 45;
  sun.shadow.camera.top = 45; sun.shadow.camera.bottom = -45;
  sun.shadow.camera.near = 20; sun.shadow.camera.far = 320;
  sun.shadow.bias = -0.0006;
  scene.add(sun);
  scene.add(sun.target);
  const hemi = new THREE.HemisphereLight(0x5a6b8a, 0x4a3b2c, 0.5);
  scene.add(hemi);

  /* ---------- environment reflections (dusk equirect) ---------- */
  {
    const c = document.createElement("canvas");
    c.width = 256; c.height = 128;
    const x = c.getContext("2d");
    const g = x.createLinearGradient(0, 0, 0, 128);
    g.addColorStop(0, "#2a3550");
    g.addColorStop(0.52, "#b07040");
    g.addColorStop(0.58, "#6a5438");
    g.addColorStop(1, "#3a3126");
    x.fillStyle = g;
    x.fillRect(0, 0, 256, 128);
    x.fillStyle = "#ffd9a8";
    x.beginPath(); x.arc(70, 62, 7, 0, Math.PI * 2); x.fill();
    const env = new THREE.CanvasTexture(c);
    env.mapping = THREE.EquirectangularReflectionMapping;
    env.colorSpace = THREE.SRGBColorSpace;
    const pmrem = new THREE.PMREMGenerator(renderer);
    scene.environment = pmrem.fromEquirectangular(env).texture;
    pmrem.dispose();
  }

  /* ---------- composer ---------- */
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), LOW ? 0.3 : 0.42, 0.5, 0.8);
  composer.addPass(bloom);
  const grade = new ShaderPass(GRADE);
  composer.addPass(grade);
  composer.addPass(new OutputPass());

  /* ---------- sky, sun sprite, stars, clouds ---------- */
  const sky = new THREE.Mesh(
    new THREE.SphereGeometry(650, 24, 16),
    new THREE.ShaderMaterial({ ...SKY, uniforms: THREE.UniformsUtils.clone(SKY.uniforms), side: THREE.BackSide, depthWrite: false, fog: false })
  );
  scene.add(sky);
  const skyU = sky.material.uniforms;

  const rand = V.mulberry(2026);
  const N_ST = LOW ? 700 : 1800;
  const stPos = new Float32Array(N_ST * 3);
  for (let i = 0; i < N_ST; i++) {
    const r = 500, th = rand() * Math.PI * 2, ph = Math.acos(1 - rand() * 0.92);
    stPos[i * 3] = r * Math.sin(ph) * Math.cos(th);
    stPos[i * 3 + 1] = r * Math.cos(ph) * 0.9 + 10;
    stPos[i * 3 + 2] = r * Math.sin(ph) * Math.sin(th);
  }
  const starGeo = new THREE.BufferGeometry();
  starGeo.setAttribute("position", new THREE.BufferAttribute(stPos, 3));
  const stars = new THREE.Points(starGeo, new THREE.PointsMaterial({ color: 0xcfd6e2, size: 1.3, transparent: true, opacity: 0, fog: false }));
  scene.add(stars);

  const GLOW = glowTex(), SMOKE = smokeTex();

  // cloud strips lit by dusk
  const clouds = [];
  for (let i = 0; i < (LOW ? 4 : 8); i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: SMOKE, color: 0xcf9a6a, transparent: true, opacity: 0.16, depthWrite: false, fog: false }));
    s.position.set(-250 + rand() * 520, 55 + rand() * 60, -420 + rand() * 180);
    s.scale.set(140 + rand() * 160, 16 + rand() * 14, 1);
    clouds.push(s);
    scene.add(s);
  }

  /* ---------- range stage ---------- */
  const range = new THREE.Group();
  scene.add(range);

  const terrGeo = new THREE.PlaneGeometry(500, 500, LOW ? 80 : 110, LOW ? 80 : 110);
  terrGeo.rotateX(-Math.PI / 2);
  {
    const pos = terrGeo.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      pos.setY(i, terrainH(pos.getX(i), pos.getZ(i)));
    }
    terrGeo.computeVertexNormals();
  }
  const terrain = new THREE.Mesh(terrGeo, new THREE.MeshStandardMaterial({ map: terrainTexture(), roughness: 0.94, metalness: 0.02 }));
  terrain.receiveShadow = true;
  range.add(terrain);

  // distant ridgeline
  for (let i = 0; i < 7; i++) {
    const w = 110 + rand() * 150, h = 26 + rand() * 34;
    const m = new THREE.Mesh(new THREE.ConeGeometry(w, h, 5, 1), new THREE.MeshBasicMaterial({ color: 0x4a3c30 }));
    const a = -0.5 + rand() * 1.6; // mostly downrange & sides
    const r = 330 + rand() * 90;
    m.position.set(Math.cos(a) * r, h / 2 - 6, Math.sin(a) * r - 60);
    m.scale.z = 0.4;
    m.rotation.y = rand();
    range.add(m);
  }

  // drifting dust
  const driftDust = [];
  for (let i = 0; i < (LOW ? 4 : 8); i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: SMOKE, color: 0xb08a5e, transparent: true, opacity: 0.05, depthWrite: false }));
    s.position.set(-40 + rand() * 120, 1.2 + rand() * 2, -30 + rand() * 80);
    s.scale.set(26 + rand() * 30, 6 + rand() * 4, 1);
    s.userData.v = 0.4 + rand() * 0.8;
    driftDust.push(s);
    range.add(s);
  }

  /* ---------- vehicles on the range ---------- */
  const howitzer = V.buildHowitzer();
  range.add(howitzer);

  const telMissile = await V.loadMissilePBR("models/hero.glb");
  const tel = V.buildTEL(telMissile);
  tel.position.set(-30, 0, 30);
  tel.rotation.y = -0.35;
  range.add(tel);

  const tank = V.buildTank();
  tank.position.set(-54, 0, -16);
  tank.rotation.y = 0.55;
  range.add(tank);

  const mlrs = V.buildMLRS();
  mlrs.position.set(-16, 0, 50);
  mlrs.rotation.y = 0.15;
  range.add(mlrs);

  const jet = V.buildJet();
  jet.visible = false;
  range.add(jet);

  // night laager light mast at the tank
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.09, 7), V.mats.metal);
  mast.position.set(-59, 3.5, -22);
  range.add(mast);
  const mastGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: GLOW, transparent: true, opacity: 0.8, depthWrite: false }));
  mastGlow.position.set(-59, 7.1, -22);
  mastGlow.scale.setScalar(1.6);
  range.add(mastGlow);
  const mastLight = new THREE.SpotLight(0xffd9a0, 0, 26, 0.7, 0.5, 1.2);
  mastLight.position.copy(mastGlow.position);
  mastLight.target.position.set(-54, 0, -16);
  range.add(mastLight, mastLight.target);

  /* ---------- FIRE MISSION fx ---------- */
  const IMPACT = V3(185, 0, -28);
  const shellCurve = new THREE.QuadraticBezierCurve3(V3(7.0, 3.6, 0), V3(92, 52, -12), V3(IMPACT.x, 2, IMPACT.z));
  const shell = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.16, 1.1, 4, 10),
    new THREE.MeshBasicMaterial({ color: 0xffc896 })
  );
  shell.visible = false;
  range.add(shell);
  const shellGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: GLOW, transparent: true, opacity: 0.9, depthWrite: false, blending: THREE.AdditiveBlending }));
  shellGlow.scale.setScalar(3.4);
  shellGlow.visible = false;
  range.add(shellGlow);

  const muzzleFlash = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTex("rgba(255,240,200,1)", "rgba(255,120,30,0)"), transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
  muzzleFlash.scale.setScalar(7);
  range.add(muzzleFlash);
  const muzzleLight = new THREE.PointLight(0xffc080, 0, 60, 1.6);
  range.add(muzzleLight);

  const dustRing = new THREE.Mesh(
    new THREE.RingGeometry(0.8, 1.35, 48),
    new THREE.MeshBasicMaterial({ map: SMOKE, color: 0xcaa878, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide })
  );
  dustRing.rotation.x = -Math.PI / 2;
  dustRing.position.set(4, 0.25, 0);
  range.add(dustRing);

  // gun smoke puffs at muzzle
  const gunSmoke = [];
  for (let i = 0; i < 9; i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: SMOKE, color: 0xb9ac9c, transparent: true, opacity: 0, depthWrite: false }));
    const a = rand() * Math.PI * 2;
    s.userData = { dx: Math.cos(a) * 0.5 + 1.2, dy: 0.25 + rand() * 0.8, dz: Math.sin(a) * 0.6, r: 0.6 + rand() };
    range.add(s);
    gunSmoke.push(s);
  }

  // impact fireball + persistent smoke column
  const fireball = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTex("rgba(255,214,150,1)", "rgba(200,60,10,0)"), transparent: true, opacity: 0, depthWrite: false, blending: THREE.AdditiveBlending }));
  fireball.position.copy(IMPACT).setY(3);
  range.add(fireball);
  const impactLight = new THREE.PointLight(0xffa050, 0, 140, 1.4);
  impactLight.position.copy(IMPACT).setY(6);
  range.add(impactLight);
  const column = [];
  for (let i = 0; i < 5; i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: SMOKE, color: 0x55504a, transparent: true, opacity: 0, depthWrite: false }));
    s.position.set(IMPACT.x + (rand() - 0.5) * 6, 4 + i * 7, IMPACT.z + (rand() - 0.5) * 6);
    s.scale.setScalar(10 + i * 5);
    column.push(s);
    range.add(s);
  }

  /* ---------- TEL launch fx ---------- */
  const telFlight = telMissile.clone(true);
  telFlight.visible = false;
  range.add(telFlight);
  const telFlame = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTex("rgba(255,235,200,1)", "rgba(255,140,40,0)"), transparent: true, opacity: 0.95, depthWrite: false, blending: THREE.AdditiveBlending }));
  telFlame.scale.setScalar(3.4);
  telFlame.visible = false;
  range.add(telFlame);
  const telLight = new THREE.PointLight(0xffc080, 0, 50, 1.6);
  range.add(telLight);
  const telSmoke = [];
  for (let i = 0; i < 10; i++) {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: SMOKE, color: 0xc0b4a4, transparent: true, opacity: 0, depthWrite: false }));
    range.add(s);
    telSmoke.push(s);
  }

  /* ---------- MLRS fx ---------- */
  const N_RK = LOW ? 3 : 6;
  const rockets = [];
  for (let i = 0; i < N_RK; i++) {
    const start = V3(-14, 4.2, 50 + (i % 3 - 1) * 0.7);
    const end = V3(150 + i * 14, 6, -60 + i * 10);
    const apex = V3(lerp(start.x, end.x, 0.45), 55 + i * 6, lerp(start.z, end.z, 0.4));
    const curve = new THREE.QuadraticBezierCurve3(start, apex, end);
    const pts = curve.getPoints(60);
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xd8cfc2, transparent: true, opacity: 0.4 })
    );
    line.geometry.setDrawRange(0, 0);
    range.add(line);
    const dart = new THREE.Sprite(new THREE.SpriteMaterial({ map: GLOW, transparent: true, opacity: 0.95, depthWrite: false, blending: THREE.AdditiveBlending }));
    dart.scale.setScalar(1.7);
    dart.visible = false;
    range.add(dart);
    rockets.push({ curve, line, dart, at: 0.22 + i * 0.09 });
  }

  /* ---------- jet fx ---------- */
  const jetCurve = new THREE.CatmullRomCurve3([
    V3(-170, 30, 110), V3(-60, 22, 45), V3(20, 16, -5), V3(110, 22, -70), V3(200, 34, -140),
  ]);
  const jetTrailPts = jetCurve.getPoints(90);
  const jetTrail = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(jetTrailPts),
    new THREE.LineBasicMaterial({ color: 0xcfd2d8, transparent: true, opacity: 0.3 })
  );
  jetTrail.geometry.setDrawRange(0, 0);
  range.add(jetTrail);
  const burner = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTex("rgba(255,210,150,1)", "rgba(255,110,40,0)"), transparent: true, opacity: 0.9, depthWrite: false, blending: THREE.AdditiveBlending }));
  burner.scale.setScalar(1.6);
  burner.visible = false;
  range.add(burner);

  /* ---------- STUDIO stage ---------- */
  const studio = new THREE.Group();
  studio.visible = false;
  scene.add(studio);
  const sFloor = new THREE.Mesh(
    new THREE.PlaneGeometry(80, 30),
    new THREE.MeshStandardMaterial({ color: 0x0b0c0e, roughness: 0.14, metalness: 0.85, envMapIntensity: 0.5, transparent: true, opacity: 0.86 })
  );
  sFloor.rotation.x = -Math.PI / 2;
  sFloor.receiveShadow = true;
  studio.add(sFloor);
  const sBack = new THREE.Mesh(new THREE.PlaneGeometry(80, 16), new THREE.MeshStandardMaterial({ color: 0x08090b, roughness: 0.9 }));
  sBack.position.set(0, 8, -9);
  studio.add(sBack);

  const sHow = V.buildHowitzer();
  const sTank = V.buildTank();
  const sTel = V.buildTEL(await V.loadMissilePBR("models/hero.glb"));
  sTel.userData.erector.rotation.z = 0.35;
  const sJet = V.buildJet();
  sJet.position.y = 1.9;
  const jetStandA = new THREE.Mesh(new THREE.BoxGeometry(0.5, 1.9, 0.5), V.mats.metal);
  jetStandA.position.set(1.5, -0.95, 0);
  const jetStandB = jetStandA.clone(); jetStandB.position.x = -2.5;
  sJet.add(jetStandA, jetStandB);
  const bays = [];
  [[sHow, -22, 0.55], [sTank, -7.5, 0.4], [sTel, 7.5, 0.35], [sJet, 22, 0.5]].forEach(([m, bx, ry]) => {
    const bay = new THREE.Group();
    bay.position.x = bx;
    m.rotation.y = ry;
    bay.add(m);
    // mirror reflection
    const mir = m.clone(true);
    mir.traverse((o) => {
      if (o.isMesh) {
        o.material = o.material.clone();
        o.material.transparent = true;
        o.material.opacity = 0.14;
        o.castShadow = o.receiveShadow = false;
      }
    });
    mir.scale.y = -1;
    bay.add(mir);
    studio.add(bay);
    bays.push(bay);
  });
  // sweeping light bar
  const bar = new THREE.Mesh(new THREE.BoxGeometry(10, 0.25, 0.8), new THREE.MeshBasicMaterial({ color: 0xfff1dd }));
  bar.position.set(0, 7.4, 2);
  studio.add(bar);
  const barLight = new THREE.SpotLight(0xfff1dd, 0, 30, 0.85, 0.45, 1.1);
  barLight.position.set(0, 7.4, 3);
  barLight.castShadow = !LOW;
  if (barLight.shadow) {
    barLight.shadow.mapSize.set(1024, 1024);
    barLight.shadow.bias = -0.0008;
  }
  studio.add(barLight, barLight.target);
  const sFill = new THREE.PointLight(0x8a94a8, 4, 40, 1.8);
  sFill.position.set(0, 5, 10);
  studio.add(sFill);

  /* ---------- per-act environment targets ---------- */
  const ENV = {
    title:      { sun: [120, 16, -40], sunI: 2.6, sunC: 0xffb46b, hemi: 0.5,  fogD: 0.0055, fogC: 0x8a6b4e, top: 0x2a3550, hor: 0xd98a4a, stars: 0.03, haze: 0.0016, glow: 1 },
    operator:   { sun: [120, 13, -36], sunI: 2.4, sunC: 0xffa95f, hemi: 0.46, fogD: 0.006,  fogC: 0x87684c, top: 0x273250, hor: 0xd3824a, stars: 0.04, haze: 0.0015,  glow: 1 },
    firemission:{ sun: [125, 11, -30], sunI: 2.3, sunC: 0xffa054, hemi: 0.42, fogD: 0.0052, fogC: 0x836549, top: 0x232e4c, hor: 0xcc7c46, stars: 0.06, haze: 0.0014, glow: 1 },
    tel:        { sun: [128, 7, -24],  sunI: 1.7, sunC: 0xff9450, hemi: 0.44, fogD: 0.0058, fogC: 0x6e5440, top: 0x1d2742, hor: 0xbd6e42, stars: 0.12, haze: 0.0011, glow: 0.9 },
    jet:        { sun: [130, 5, -18],  sunI: 1.15, sunC: 0xff8445, hemi: 0.28, fogD: 0.005, fogC: 0x5c4738, top: 0x182138, hor: 0xa95f3c, stars: 0.3,  haze: 0.0008, glow: 0.8 },
    mlrs:       { sun: [132, 2.5, -12], sunI: 0.5, sunC: 0xff7440, hemi: 0.2, fogD: 0.0048, fogC: 0x403428, top: 0x111a30, hor: 0x7c492f, stars: 0.55, haze: 0.0005, glow: 0.55 },
    night:      { sun: [-70, 46, 50],  sunI: 0.3, sunC: 0x8094c2, hemi: 0.17, fogD: 0.0042, fogC: 0x0d1016, top: 0x070b18, hor: 0x141a26, stars: 1,    haze: 0,      glow: 0 },
    studio:     { sun: [0, 60, 0],     sunI: 0.0, sunC: 0x8094c2, hemi: 0.05, fogD: 0.012,  fogC: 0x060708, top: 0x030405, hor: 0x07080a, stars: 0,    haze: 0,      glow: 0 },
    comms:      { sun: [-70, 46, 50],  sunI: 0.26, sunC: 0x8094c2, hemi: 0.1, fogD: 0.0038, fogC: 0x0c0f15, top: 0x060a16, hor: 0x121826, stars: 1,    haze: 0,      glow: 0 },
  };
  const cur = {
    sun: V3(120, 16, -40), sunC: new THREE.Color(0xffb46b), fogC: new THREE.Color(0x8a6b4e),
    top: new THREE.Color(0x2a3550), hor: new THREE.Color(0xd98a4a),
  };
  const tgt = { sunC: new THREE.Color(), fogC: new THREE.Color(), top: new THREE.Color(), hor: new THREE.Color(), sun: V3() };
  const tints = {
    warm: new THREE.Vector3(1.04, 0.99, 0.94),
    dusk: new THREE.Vector3(1.05, 0.97, 0.92),
    night: new THREE.Vector3(0.92, 0.96, 1.08),
    flat: new THREE.Vector3(1, 1, 1),
  };
  const tgtTint = tints.warm.clone();

  /* ---------- helpers ---------- */
  const tmp = V3(), tmp2 = V3(), tan = V3();
  const muzzleWorld = () => {
    howitzer.updateWorldMatrix(true, false);
    return tmp.copy(howitzer.userData.muzzleLocal).applyMatrix4(
      (howitzer.userData.recoil.updateWorldMatrix(true, false), howitzer.userData.recoil.matrixWorld)
    );
  };

  const QF = 0.34;   // howitzer fire
  const QT = 0.62;   // TEL fire

  /* ---------- acts ---------- */
  const ACTS = {
    title: {
      run(q, t) {
        vLerp(camPos, tmp.set(26, 2.2, 30), tmp2.set(15, 2.0, 18.5), smooth(q));
        camLook.set(0, 3.1, 0);
        howitzer.userData.barrelPivot.rotation.z = 0.14;
        tgtTint.copy(tints.warm);
      },
    },
    operator: {
      run(q, t) {
        vLerp(camPos, tmp.set(8.5, 1.9, 8), tmp2.set(4.2, 2.3, 5.6), smooth(q));
        vLerp(camLook, tmp.set(1.5, 1.8, 0), tmp2.set(-0.5, 2.2, 0), smooth(q));
        howitzer.userData.barrelPivot.rotation.z = 0.14;
        tgtTint.copy(tints.warm);
      },
    },
    firemission: {
      run(q, t) {
        const elev = 0.14 + 0.58 * smooth(clamp01((q - 0.04) / 0.24));
        howitzer.userData.barrelPivot.rotation.z = elev;
        howitzer.userData.recoil.position.x = -0.85 * gauss(q, QF + 0.012, 0.035);
        const mw = muzzleWorld();
        muzzleFlash.position.copy(mw);
        muzzleFlash.material.opacity = gauss(q, QF, 0.012);
        muzzleLight.position.copy(mw);
        muzzleLight.intensity = 90 * gauss(q, QF, 0.02);
        grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.6 * gauss(q, QF, 0.01));
        world.shake = Math.max(world.shake, 0.55 * gauss(q, QF + 0.012, 0.04));
        // ground shock ring + smoke
        const rk = clamp01((q - QF) / 0.2);
        dustRing.scale.setScalar(1 + rk * 26);
        dustRing.material.opacity = rk > 0 ? (1 - rk) * 0.55 : 0;
        for (const s of gunSmoke) {
          const u = s.userData;
          const k = clamp01((q - QF) / (0.3 * u.r));
          s.position.copy(mw).add(tmp2.set(u.dx * k * 4, u.dy * k * 3, u.dz * k * 4));
          s.scale.setScalar(1 + k * 5 * u.r);
          s.material.opacity = k > 0 ? Math.pow(1 - k, 1.4) * 0.4 : 0;
        }
        // shell + camera
        if (q < QF) {
          const k = smooth(q / QF);
          vLerp(camPos, tmp.set(-6.5, 2.1, 8.5), tmp2.set(-4.2, 2.8, 6.2), k);
          camLook.set(5, 2.8 + elev * 3, 0);
          shell.visible = shellGlow.visible = false;
        } else if (q < 0.86) {
          const s = (q - QF) / (0.86 - QF);
          shellCurve.getPoint(s, shell.position);
          shellCurve.getTangent(s, tan);
          shell.quaternion.setFromUnitVectors(V3(0, 1, 0), tan);
          shell.visible = !LOW || s < 0.9;
          shellGlow.position.copy(shell.position);
          shellGlow.visible = true;
          const chase = smooth(clamp01(s * 6)); // whip onto the shell
          vLerp(camPos, tmp.set(-4.2, 2.8, 6.2), tmp2.copy(shell.position).add(tan.clone().multiplyScalar(-10)).add(V3(0, 1.6, 4.2)), chase);
          vLerp(camLook, tmp.set(5, 4.5, 0), shell.position, chase);
        } else {
          shell.visible = shellGlow.visible = false;
          const e = clamp01((q - 0.86) / 0.14);
          camPos.copy(V3(118, 9, 26));
          camLook.copy(IMPACT).setY(6);
          fireball.material.opacity = Math.pow(1 - e, 1.2);
          fireball.scale.setScalar(5 + Math.pow(e, 0.4) * 34);
          impactLight.intensity = 320 * gauss(e, 0.08, 0.2);
          grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.5 * gauss(q, 0.868, 0.012));
          world.shake = Math.max(world.shake, 0.4 * gauss(q, 0.88, 0.05));
          for (let i = 0; i < column.length; i++) {
            column[i].material.opacity = clamp01(e * 2 - i * 0.18) * 0.65;
          }
        }
        tgtTint.copy(tints.dusk);
      },
    },
    tel: {
      run(q, t) {
        const base = tel.position;
        const er = tel.userData.erector;
        er.rotation.z = (Math.PI / 2) * smooth(clamp01((q - 0.06) / 0.4));
        const launched = q >= QT;
        tel.userData.missile.visible = !launched;
        telFlight.visible = launched && q < 0.93;
        const tipW = tmp.set(-30 + 0.6 * Math.cos(er.rotation.z - 0.35), 1.55 + 4.2 * Math.sin(er.rotation.z + 0.35) + 1.2, 30 - 0.6);
        if (launched) {
          const fp = Math.pow(clamp01((q - QT) / 0.33), 1.6);
          telFlight.position.set(base.x + 1 + fp * 9, 4.6 + fp * 85, base.z - 1 - fp * 6);
          telFlight.rotation.z = -0.12 * fp;
          telFlame.visible = true;
          telFlame.position.copy(telFlight.position).add(tmp2.set(-0.4 * fp, -3, 0));
          telLight.position.copy(telFlame.position);
          telLight.intensity = 50 * clamp01(1.2 - fp) + 80 * gauss(q, QT, 0.02);
          grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.45 * gauss(q, QT, 0.012));
          world.shake = Math.max(world.shake, 0.4 * gauss(q, QT + 0.015, 0.045));
        } else {
          telFlame.visible = false;
          telLight.intensity = 0;
        }
        for (let i = 0; i < telSmoke.length; i++) {
          const s = telSmoke[i];
          const k = clamp01((q - QT) / 0.34) * (0.5 + (i % 5) * 0.14);
          const a = i * 0.63;
          s.position.set(base.x + 1 + Math.cos(a) * (1 + k * 7), 1 + k * 5, base.z - 1 + Math.sin(a) * (1 + k * 7));
          s.scale.setScalar(2 + k * 7);
          s.material.opacity = q > QT && k < 1 ? Math.pow(1 - k, 1.5) * 0.45 : 0;
        }
        if (q < QT) {
          // from behind-left so the erecting missile silhouettes against the sunset
          vLerp(camPos, tmp.set(base.x - 15, 2.8, base.z + 12), tmp2.set(base.x - 9.5, 2.4, base.z + 8.5), smooth(q / QT));
          camLook.set(base.x + 1, 2.6 + er.rotation.z * 3.2, base.z - 1);
        } else {
          const fp = clamp01((q - QT) / 0.33);
          camPos.set(base.x - 10, 2.6 + fp * 4, base.z + 9);
          vLerp(camLook, tmp.set(base.x + 1, 7, base.z - 1), telFlight.position, smooth(clamp01(fp * 1.6)));
        }
        tgtTint.copy(tints.dusk);
      },
    },
    jet: {
      run(q, t) {
        jet.visible = true;
        const s = clamp01(q * 0.96 + 0.02);
        jetCurve.getPoint(s, jet.position);
        jetCurve.getTangent(s, tan);
        jet.quaternion.setFromUnitVectors(V3(1, 0, 0), tan);
        jet.rotateX(Math.sin(s * Math.PI * 2) * 0.5);
        jetTrail.geometry.setDrawRange(0, Math.floor(s * 90));
        burner.visible = true;
        burner.position.copy(jet.position).addScaledVector(tan, -5.4);
        camPos.set(12, 1.6, -4);
        camLook.copy(jet.position);
        world.shake = Math.max(world.shake, 0.12 * gauss(s, 0.52, 0.08));
        tgtTint.copy(tints.dusk);
      },
    },
    mlrs: {
      run(q, t) {
        const base = mlrs.position;
        for (const r of rockets) {
          const pr = clamp01((q - r.at) / 0.42);
          r.line.geometry.setDrawRange(0, Math.floor(pr * 60));
          r.dart.visible = pr > 0 && pr < 1;
          if (r.dart.visible) r.curve.getPoint(pr, r.dart.position);
          world.shake = Math.max(world.shake, 0.3 * gauss(q, r.at + 0.012, 0.02));
          grade.uniforms.uFlash.value = Math.max(grade.uniforms.uFlash.value, 0.22 * gauss(q, r.at, 0.008));
        }
        vLerp(camPos, tmp.set(base.x + 11, 2.6, base.z + 9), tmp2.set(base.x + 6.5, 3.4, base.z + 5), smooth(q));
        camLook.set(base.x - 1, 3.4, base.z - 2);
        tgtTint.copy(tints.dusk).lerp(tints.night, q * 0.5);
      },
    },
    night: {
      run(q, t) {
        mastLight.intensity = 70;
        const base = tank.position;
        vLerp(camPos, tmp.set(base.x + 12.5, 2.8, base.z + 9.5), tmp2.set(base.x + 8, 2.2, base.z + 6.5), smooth(q));
        camLook.set(base.x, 1.7, base.z);
        tank.userData.turret.rotation.y = Math.sin(t * 0.1) * 0.3;
        tgtTint.copy(tints.night);
      },
    },
    studio: {
      run(q, t) {
        mastLight.intensity = 0;
        const dolly = lerp(-26, 26, smooth(Math.min(q / 0.88, 1)));
        if (q < 0.88) {
          camPos.set(dolly, 2.5, 12.5);
          camLook.set(dolly + 1.2, 1.6, 0);
        } else {
          const b = smooth((q - 0.88) / 0.12);
          vLerp(camPos, tmp.set(26, 2.5, 12.5), tmp2.set(0, 4.6, 19), b);
          vLerp(camLook, tmp.set(27.2, 1.6, 0), tmp2.set(0, 1.4, 0), b);
        }
        bar.position.x = dolly + 2;
        barLight.position.set(dolly + 2, 7.4, 3);
        barLight.target.position.set(dolly + 2, 0, 0);
        barLight.intensity = 240;
        tgtTint.copy(tints.flat);
      },
    },
    comms: {
      run(q, t) {
        barLight.intensity = 0;
        vLerp(camPos, tmp.set(19, 3.2, 25), tmp2.set(13, 2.7, 17.5), smooth(q));
        camLook.set(0, 2.2, 0);
        howitzer.userData.barrelPivot.rotation.z = 0.5;
        tgtTint.copy(tints.night);
      },
    },
  };

  /* ---------- ranges + frame loop ---------- */
  const ids = ["title", "operator", "firemission", "tel", "jet", "mlrs", "night", "studio", "comms"];
  let ranges = ids.map((id, i) => ({ id, start: i / 9, end: (i + 1) / 9 }));
  let fireActEnd = 3 / 9;

  const world = {
    renderer,
    shake: 0,
    pointer: { x: 0, y: 0 },
    currentAct: "title",
    setActRanges(r) {
      ranges = r;
      const f = r.find((x) => x.id === "firemission");
      if (f) fireActEnd = f.end;
    },
    degrade() {
      renderer.shadowMap.enabled = false;
      renderer.setPixelRatio(1);
      renderer.setSize(innerWidth, innerHeight);
      composer.setPixelRatio(1);
      composer.setSize(innerWidth, innerHeight);
      bloom.strength *= 0.7;
    },
    resize() {
      camera.aspect = innerWidth / innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(innerWidth, innerHeight);
      composer.setSize(innerWidth, innerHeight);
    },
    update(p, dt, t) {
      let act = ranges[0];
      for (let i = ranges.length - 1; i >= 0; i--) {
        if (p >= ranges[i].start) { act = ranges[i]; break; }
      }
      const q = clamp01((p - act.start) / Math.max(act.end - act.start, 1e-4));
      world.currentAct = act.id;
      range.visible = act.id !== "studio";
      studio.visible = act.id === "studio";

      world.shake = Math.max(0, world.shake - dt * 0.8);
      grade.uniforms.uFlash.value = Math.max(0, grade.uniforms.uFlash.value - dt * 2.6);
      (ACTS[act.id] || ACTS.title).run(q, t);

      // never let the camera sink into a dune
      if (act.id !== "studio") {
        const gH = terrainH(camPos.x, camPos.z) + 1.5;
        if (camPos.y < gH) camPos.y = gH;
      }
      mastGlow.material.opacity = act.id === "night" || act.id === "comms" ? 0.8 : 0;

      // aftermath smoke column persists once the fire mission is behind us
      const after = clamp01((p - fireActEnd) * 30) * (act.id === "studio" ? 0 : 1);
      if (act.id !== "firemission") {
        for (let i = 0; i < column.length; i++) column[i].material.opacity = after * 0.3;
        fireball.material.opacity = 0;
        impactLight.intensity = 0;
      }
      if (act.id !== "jet") { jet.visible = false; burner.visible = false; }
      if (act.id !== "tel") { telFlame.visible = false; telFlight.visible = false; telLight.intensity = 0; if (tel.userData.missile) tel.userData.missile.visible = true; }
      if (act.id !== "firemission") { shell.visible = shellGlow.visible = false; muzzleFlash.material.opacity = 0; muzzleLight.intensity = 0; dustRing.material.opacity = 0; for (const s of gunSmoke) s.material.opacity = 0; }

      // environment lerp
      const E = ENV[act.id] || ENV.title;
      const el = 1 - Math.pow(0.002, dt);
      tgt.sun.set(E.sun[0], E.sun[1], E.sun[2]);
      cur.sun.lerp(tgt.sun, el);
      sun.position.copy(cur.sun);
      sun.intensity = lerp(sun.intensity, E.sunI, el);
      sun.color.lerp(tgt.sunC.setHex(E.sunC), el);
      hemi.intensity = lerp(hemi.intensity, E.hemi, el);
      scene.fog.density = lerp(scene.fog.density, E.fogD, el);
      scene.fog.color.lerp(tgt.fogC.setHex(E.fogC), el);
      skyU.uTop.value.lerp(tgt.top.setHex(E.top), el);
      skyU.uHorizon.value.lerp(tgt.hor.setHex(E.hor), el);
      skyU.uSunDir.value.copy(cur.sun).normalize();
      skyU.uSunGlow.value = lerp(skyU.uSunGlow.value, E.glow, el);
      stars.material.opacity = lerp(stars.material.opacity, E.stars, el);
      grade.uniforms.uHaze.value = lerp(grade.uniforms.uHaze.value, E.haze, el);
      grade.uniforms.uTint.value.lerp(tgtTint, el);
      grade.uniforms.uTime.value = t % 20;
      for (const c of clouds) c.material.opacity = lerp(c.material.opacity, E.stars > 0.8 ? 0.02 : 0.15, el);

      // ambient motion
      for (const s of driftDust) {
        s.position.x += dt * s.userData.v;
        if (s.position.x > 90) s.position.x = -50;
      }
      sun.target.position.set(camLook.x, 0, camLook.z);

      // camera: handheld drift + parallax + shake
      const hx = Math.sin(t * 0.9) * 0.05 + Math.sin(t * 2.3 + 1) * 0.03;
      const hy = Math.sin(t * 1.3 + 2) * 0.04 + Math.sin(t * 3.1) * 0.02;
      const shx = (Math.sin(t * 41) * 0.6 + Math.sin(t * 59 + 1.3) * 0.4) * world.shake;
      const shy = (Math.sin(t * 47 + 2.1) * 0.6 + Math.sin(t * 33) * 0.4) * world.shake;
      camera.position.set(
        camPos.x + hx + world.pointer.x * 0.4 + shx,
        camPos.y + hy - world.pointer.y * 0.25 + shy,
        camPos.z
      );
      camera.lookAt(camLook);
      composer.render();
    },
  };
  world._debug = { scene, camera, THREE, composer, renderer, range, sky, bloom, grade };
  return world;
}
