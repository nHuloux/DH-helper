// viewer.js — scène three.js (Z vers le haut, unités mm) : segments du robot, axes articulaires, repères DH.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { mat4Mul, mat4InvRigid, mat4Point, mat4Vec, cross, unit, norm, sub } from './linalg.js';
import { geometriePrimitive } from './meshes.js';

const PALETTE = ['#9aa1a9', '#dfe3e8', '#c4c9d0', '#dfe3e8', '#c4c9d0', '#dfe3e8', '#c4c9d0', '#dfe3e8', '#c4c9d0'];
const COULEURS_REPERE = { x: '#d62728', y: '#2ca02c', z: '#1f4fd6' };
const SUB = '₀₁₂₃₄₅₆₇₈₉';
const indice = (n) => String(n).split('').map((c) => SUB[+c]).join('');
const M4 = (A) => new THREE.Matrix4().set(...A);
const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

function etiquette(texte, couleur, taille, bg = false) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 96;
  const g = c.getContext('2d');
  g.font = 'bold 52px system-ui, Segoe UI, Roboto, sans-serif';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  if (bg) { g.fillStyle = 'rgba(255,255,255,.8)'; g.beginPath(); g.roundRect(8, 8, 240, 80, 22); g.fill(); }
  g.fillStyle = couleur; g.fillText(texte, 128, 50);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false }));
  sp.scale.set(taille * 2.67, taille, 1);
  sp.renderOrder = 10;
  return sp;
}

function liberer(obj) {
  obj.traverse((o) => {
    if (o.geometry && !o.userData.geoPartagee) o.geometry.dispose();
    if (o.material) { const m = Array.isArray(o.material) ? o.material : [o.material]; m.forEach((mm) => { if (mm.map) mm.map.dispose(); mm.dispose(); }); }
  });
}

export class Viewer {
  constructor(conteneur, { auSurvol = null } = {}) {
    this.el = conteneur;
    this.auSurvol = auSurvol;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0xffffff, 1);
    this.el.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(35, 1, 5, 100000);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.addEventListener('change', () => this.demander());
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
    const d1 = new THREE.DirectionalLight(0xffffff, 1.5); d1.position.set(800, -1200, 1500); this.scene.add(d1);
    const d2 = new THREE.DirectionalLight(0xffffff, 0.6); d2.position.set(-900, 700, 400); this.scene.add(d2);
    this.racine = new THREE.Group(); this.scene.add(this.racine);
    this.lignes = [];                       // matériaux Line2 (résolution à mettre à jour)
    this.opacite = 0.45; this.afficherAxes = true; this.afficherReperes = true;
    this.groupes = []; this.grAxes = []; this.grReperes = []; this.grFixes = null;
    this.ray = new THREE.Raycaster(); this.souris = new THREE.Vector2();
    this._rafDemande = false;
    new ResizeObserver(() => this.redimensionner()).observe(this.el);
    this.renderer.domElement.addEventListener('pointermove', (e) => this._survol(e));
    this.renderer.domElement.addEventListener('pointerleave', () => this.auSurvol && this.auSurvol(null));
    this.redimensionner();
  }

  demander() {
    if (this._rafDemande) return;
    this._rafDemande = true;
    requestAnimationFrame(() => { this._rafDemande = false; this.renderer.render(this.scene, this.camera); });
  }
  redimensionner() {
    const w = Math.max(this.el.clientWidth, 10), h = Math.max(this.el.clientHeight, 10);
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    this.lignes.forEach((m) => m.resolution.set(w, h));
    this.demander();
  }
  _materielLigne(couleur, largeur, tirets = null) {
    const m = new LineMaterial({ color: new THREE.Color(couleur), linewidth: largeur, dashed: !!tirets, transparent: true, depthTest: true });
    if (tirets) { m.dashSize = tirets[0]; m.gapSize = tirets[1]; }
    m.resolution.set(this.el.clientWidth || 800, this.el.clientHeight || 600);
    this.lignes.push(m);
    return m;
  }
  _ligne(a, b, couleur, largeur, tirets = null) {
    const g = new LineGeometry(); g.setPositions([...a, ...b]);
    const l = new Line2(g, this._materielLigne(couleur, largeur, tirets));
    if (tirets) l.computeLineDistances();
    return l;
  }

  // ---------------------------------------------------------------- robot
  // geoms : tableau aligné sur robot.visuels (BufferGeometry en mm, ou null → primitive/absent)
  setRobot(robot, geoms) {
    this.vider();
    this.robot = robot;
    this.groupes = [];
    for (let m = 0; m <= robot.n; m++) { const g = new THREE.Group(); g.matrixAutoUpdate = false; this.racine.add(g); this.groupes.push(g); }
    this.materiaux = [];
    robot.visuels.forEach((v, idx) => {
      let geo = geoms[idx] || null, partagee = true;
      if (!geo && v.type !== 'mesh') { geo = geometriePrimitive(v); partagee = false; }
      if (!geo) return;
      const mat = new THREE.MeshStandardMaterial({ color: PALETTE[v.m % PALETTE.length], roughness: 0.55, metalness: 0.1, side: THREE.DoubleSide, transparent: true, opacity: this.opacite, depthWrite: this.opacite > 0.98 });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.userData.geoPartagee = partagee; mesh.userData.segment = v.m;
      mesh.matrixAutoUpdate = false; mesh.matrix.copy(M4(v.T));
      this.groupes[v.m].add(mesh);
      this.materiaux.push(mat);
    });
    this.sansMaillage = this.materiaux.length === 0;
    // squelette si aucun maillage n'a pu être chargé : traits entre les origines des articulations
    this.grFixes = new THREE.Group(); this.racine.add(this.grFixes);
    this.grAxes = []; this.grReperes = [];
    for (let k = 0; k <= robot.n; k++) {
      const a = new THREE.Group(); a.matrixAutoUpdate = false; this.racine.add(a); this.grAxes.push(a);
      const r = new THREE.Group(); r.matrixAutoUpdate = false; this.racine.add(r); this.grReperes.push(r);
    }
    const L = robot.taille;
    const grille = new THREE.GridHelper(L * 2.4, 24, 0xbbbbbb, 0xe3e3e3);
    grille.rotation.x = Math.PI / 2; grille.userData.fixe = true;
    this.racine.add(grille); this.grille = grille;
    this.F = robot.F0.map((m) => m.slice());
    this.construireAxes();
    this.cadrer();
    this.demander();
  }

  vider() {
    if (this.racine) { liberer(this.racine); this.racine.clear(); }
    this.lignes = [];
  }

  cadrer() {
    const robot = this.robot;
    this.racine.updateMatrixWorld(true);
    const boite = new THREE.Box3();
    this.groupes.forEach((g) => { if (g.children.length) boite.expandByObject(g); });
    robot.F0.forEach((m) => boite.expandByPoint(new THREE.Vector3(m[3], m[7], m[11])));
    boite.expandByPoint(V3(robot.OUTIL));
    const c = boite.getCenter(new THREE.Vector3()), r = Math.max(boite.getSize(new THREE.Vector3()).length() / 2, 150);
    const dist = r / Math.sin((this.camera.fov * Math.PI) / 360) * 1.05;
    const dir = new THREE.Vector3(0.62, -0.72, 0.42).normalize();
    this.controls.target.copy(c);
    this.camera.position.copy(c).addScaledVector(dir, dist);
    this.camera.near = Math.max(dist / 200, 1); this.camera.far = dist * 40; this.camera.updateProjectionMatrix();
    this.controls.update();
    this.demander();
  }

  // ---------------------------------------------------------------- axes articulaires (fixes dans leur segment)
  construireAxes(courant = 1) {
    const robot = this.robot; if (!robot) return;
    this.courantAxe = courant;
    for (let k = 0; k <= robot.n; k++) { liberer(this.grAxes[k]); this.grAxes[k].clear(); }
    const L = robot.taille * 0.55;
    for (let k = 0; k < robot.n; k++) {
      const { p, u } = robot.AXES[k];
      const sel = (k === Math.min(courant, robot.n - 1));
      const a = [p[0] - L * u[0], p[1] - L * u[1], p[2] - L * u[2]], b = [p[0] + L * u[0], p[1] + L * u[1], p[2] + L * u[2]];
      const ln = this._ligne(a, b, sel ? '#e0a800' : '#FFCF00', sel ? 4 : 2.5, [L / 30, L / 60]);
      ln.userData.axe = k;
      this.grAxes[k].add(ln);
      const s = etiquette(`${robot.types && robot.types[k] === 'P' ? 'prism.' : 'art.'} ${k + 1}`, '#a07800', robot.taille * 0.045, true);
      s.position.set(b[0] + u[0] * robot.taille * 0.03, b[1] + u[1] * robot.taille * 0.03, b[2] + u[2] * robot.taille * 0.03);
      this.grAxes[k].add(s);
    }
    this._appliquerVisibilites();
    this.appliquerPose(this.F);
  }

  // ---------------------------------------------------------------- repères DH
  setReperes(reperes, courant) {
    const robot = this.robot; if (!robot) return;
    for (const g of this.grReperes) { liberer(g); g.clear(); }
    const L0 = robot.taille * 0.09;
    (reperes || []).forEach((r, k) => {
      const sel = k === courant;
      const L = sel ? L0 * 1.35 : L0;
      const O = r.O, x = unit(r.x), z = unit(r.z), y = cross(z, x);
      const g = this.grReperes[k];
      [['x', x], ['y', y], ['z', z]].forEach(([nom, v]) => {
        const c = COULEURS_REPERE[nom];
        const fin = [O[0] + L * v[0], O[1] + L * v[1], O[2] + L * v[2]];
        g.add(this._ligne(O, fin, c, sel ? 5 : 3.5));
        const cone = new THREE.Mesh(new THREE.ConeGeometry(L * 0.07, L * 0.2, 14), new THREE.MeshBasicMaterial({ color: c }));
        cone.position.set(fin[0], fin[1], fin[2]).addScaledVector(V3(v), L * 0.1);
        cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), V3(v));
        g.add(cone);
        const s = etiquette(nom + indice(k), c, robot.taille * (sel ? 0.05 : 0.04));
        s.position.set(O[0] + 1.32 * L * v[0], O[1] + 1.32 * L * v[1], O[2] + 1.32 * L * v[2]);
        g.add(s);
      });
      const bille = new THREE.Mesh(new THREE.SphereGeometry(L * (sel ? 0.1 : 0.07), 16, 12), new THREE.MeshBasicMaterial({ color: sel ? '#e0a800' : '#363636' }));
      bille.position.set(O[0], O[1], O[2]); bille.userData.origine = k;
      g.add(bille);
    });
    this._appliquerVisibilites();
    this.appliquerPose(this.F);
    this.demander();
  }

  // ---------------------------------------------------------------- pose
  appliquerPose(F) {
    if (!this.robot || !F) return;
    this.F = F;
    F.forEach((m, k) => this.groupes[k] && this.groupes[k].matrix.copy(M4(m)));
    for (let k = 0; k <= this.robot.n; k++) {
      const Mk = mat4Mul(F[k], mat4InvRigid(this.robot.F0[k]));
      const mk = M4(Mk);
      if (this.grAxes[k]) this.grAxes[k].matrix.copy(mk);
      if (this.grReperes[k]) this.grReperes[k].matrix.copy(mk);
    }
    this.racine.updateMatrixWorld(true);
    this.demander();
  }

  _appliquerVisibilites() {
    this.grAxes.forEach((g) => { g.visible = this.afficherAxes; });
    this.grReperes.forEach((g) => { g.visible = this.afficherReperes; });
  }
  montrerAxes(v) { this.afficherAxes = v; this._appliquerVisibilites(); this.demander(); }
  montrerReperes(v) { this.afficherReperes = v; this._appliquerVisibilites(); this.demander(); }
  setOpacite(o) {
    this.opacite = o;
    (this.materiaux || []).forEach((m) => { m.opacity = o; m.depthWrite = o > 0.98; m.needsUpdate = true; });
    this.demander();
  }

  // ---------------------------------------------------------------- survol : coordonnées du point visé
  _survol(e) {
    if (!this.auSurvol || !this.robot) return;
    const r = this.renderer.domElement.getBoundingClientRect();
    this.souris.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.ray.setFromCamera(this.souris, this.camera);
    const cibles = [];
    this.groupes.forEach((g) => g.traverse((o) => { if (o.isMesh) cibles.push(o); }));
    const hit = this.ray.intersectObjects(cibles, false)[0];
    this.auSurvol(hit ? [hit.point.x, hit.point.y, hit.point.z] : null);
  }

  // ---------------------------------------------------------------- image
  png() {
    this.renderer.render(this.scene, this.camera);
    return new Promise((ok) => this.renderer.domElement.toBlob(ok, 'image/png'));
  }
  detruire() { this.vider(); this.renderer.dispose(); this.renderer.domElement.remove(); }
}
