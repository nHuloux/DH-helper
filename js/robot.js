// robot.js — URDF -> chaîne série, axes articulaires, repères DH (convention standard), analyse des repères d'un étudiant.
// Unités : mm et degrés. Aucune dépendance (le DOM est passé en paramètre : DOMParser dans le navigateur, xmldom sous Node).
import {
  add, sub, mul, dot, cross, norm, unit, neg, deg, rad,
  I4, mat4Mul, mat4InvRigid, mat4Point, mat4Vec, mat4Trans, mat4From, mat3Mul, Rx, Ry, Rz, rotAxe,
  projDroite, distDroite, angleSigne, piedsNormaleCommune,
} from './linalg.js';

// ------------------------------------------------------------------ lecture XML
const enfants = (el, tag) => Array.from(el.childNodes).filter((n) => n.nodeType === 1 && n.nodeName === tag);
const premier = (el, tag) => (el ? enfants(el, tag)[0] || null : null);
const vec = (s, defaut) => (s && s.trim() ? s.trim().split(/\s+/).map(Number) : defaut.slice());
const attr = (el, nom) => (el && el.hasAttribute && el.hasAttribute(nom) ? el.getAttribute(nom) : null);

function origine(el) {
  const o = premier(el, 'origin');
  const xyz = vec(attr(o, 'xyz'), [0, 0, 0]);
  const rpy = vec(attr(o, 'rpy'), [0, 0, 0]);
  const R = mat3Mul(Rz(rpy[2]), mat3Mul(Ry(rpy[1]), Rx(rpy[0])));
  return mat4From(R, xyz);
}
const enMm = (T) => { const U = T.slice(); U[3] *= 1000; U[7] *= 1000; U[11] *= 1000; return U; };

// ------------------------------------------------------------------ analyse d'un URDF (branches possibles)
export function lireURDF(doc) {
  const root = doc.documentElement;
  if (!root || root.nodeName !== 'robot') throw new Error("Ce fichier n'est pas un URDF (balise <robot> absente).");
  const nom = attr(root, 'name') || 'robot';
  const liens = {};
  enfants(root, 'link').forEach((l) => { liens[attr(l, 'name')] = l; });
  const joints = enfants(root, 'joint').map((j) => {
    const lim = premier(j, 'limit'), ax = premier(j, 'axis');
    return {
      name: attr(j, 'name'), type: attr(j, 'type'), parent: attr(premier(j, 'parent'), 'link'), child: attr(premier(j, 'child'), 'link'),
      T: origine(j), axe: vec(attr(ax, 'xyz'), [1, 0, 0]),
      lo: lim && attr(lim, 'lower') !== null ? Number(attr(lim, 'lower')) : null,
      hi: lim && attr(lim, 'upper') !== null ? Number(attr(lim, 'upper')) : null,
    };
  });
  return { root, nom, liens, joints };
}

// tolérances géométriques : les URDF arrondissent souvent les angles (1.5708 au lieu de pi/2) → ~1e-5 rad de bruit
export const TOL_PARALLELE = 2e-3; // |u1 × u2| sous lequel deux axes sont parallèles (~0,11°)
export const TOL_SECANT = 0.1; // mm : distance entre axes sous laquelle ils sont sécants

const MOBILES = ['revolute', 'continuous', 'prismatic'];
const ROT = ['revolute', 'continuous'];

// Toutes les chaînes racine -> feuille, tronquées après la dernière rotoïde (+ articulations fixes qui suivent)
export function brancheDisponibles(urdf, base = null) {
  const { liens, joints } = urdf;
  const parParent = {};
  joints.forEach((j) => { (parParent[j.parent] = parParent[j.parent] || []).push(j); });
  const fils = new Set(joints.map((j) => j.child));
  const racines = Object.keys(liens).filter((n) => !fils.has(n));
  if (!base) {
    if (!racines.length) throw new Error("Aucun lien racine dans l'URDF.");
    base = racines[0];
  }
  const chemins = [];
  const dfs = (lien, acc) => {
    const suite = parParent[lien] || [];
    if (!suite.length) chemins.push(acc.slice());
    suite.forEach((j) => { acc.push(j); dfs(j.child, acc); acc.pop(); });
  };
  dfs(base, []);
  const branches = chemins.map((ch) => {
    let derniere = -1;
    ch.forEach((j, k) => { if (ROT.includes(j.type)) derniere = k; });
    if (derniere < 0) return { tip: ch.length ? ch[ch.length - 1].child : base, nRot: 0, valide: false, raison: 'aucune articulation rotoïde', joints: ch };
    let fin = derniere;
    while (fin + 1 < ch.length && ch[fin + 1].type === 'fixed') fin++;
    const util = ch.slice(0, fin + 1);
    const mauvais = util.find((j) => !['fixed', ...ROT].includes(j.type));
    return {
      tip: util[util.length - 1].child, nRot: util.filter((j) => ROT.includes(j.type)).length, joints: util,
      valide: !mauvais, raison: mauvais ? `articulation « ${mauvais.name} » de type ${mauvais.type} non gérée (rotoïdes uniquement)` : '',
    };
  });
  // dédoublonnage (plusieurs feuilles qui partagent la même chaîne utile)
  const vus = new Set();
  const uniques = branches.filter((b) => { const k = b.joints.map((j) => j.name).join('>'); if (vus.has(k)) return false; vus.add(k); return true; });
  const bonnes = uniques.filter((b) => b.valide);
  const defaut = bonnes.length ? bonnes.reduce((a, b) => (b.nRot > a.nRot ? b : a)) : null;
  return { base, branches: uniques, defaut };
}

// ------------------------------------------------------------------ le robot
export class Robot {
  constructor(urdf, { tip = null, base = null } = {}) {
    this.nom = urdf.nom;
    const info = brancheDisponibles(urdf, base);
    let br = tip ? info.branches.find((b) => b.tip === tip) : info.defaut;
    if (!br) throw new Error(tip ? `Lien terminal « ${tip} » introuvable.` : 'Aucune chaîne exploitable (articulations rotoïdes en série) dans cet URDF.');
    if (!br.valide) throw new Error(`Chaîne non gérée : ${br.raison}.`);
    this.infoBranches = info;
    this.base = info.base;
    this.tip = br.tip;
    this.ORIG = []; this.AXL = []; this.noms = []; this.limites = [];
    const attache = [{ lien: info.base, m: 0, Tl: I4() }];
    let Tacc = I4();
    let m = 0;
    for (const j of br.joints) {
      const To = enMm(j.T);
      if (j.type === 'fixed') {
        Tacc = mat4Mul(Tacc, To);
        attache.push({ lien: j.child, m, Tl: Tacc.slice() });
      } else {
        this.ORIG.push(mat4Mul(Tacc, To));
        this.AXL.push(unit(j.axe));
        this.noms.push(j.name);
        this.limites.push(j.type === 'continuous' || j.lo === null || j.hi === null ? [-180, 180] : [deg(j.lo), deg(j.hi)]);
        m++;
        Tacc = I4();
        attache.push({ lien: j.child, m, Tl: I4() });
      }
    }
    this.n = m;
    this.TIP = Tacc;
    this.F0 = this.fk(new Array(this.n).fill(0));
    this.AXES = [];
    for (let k = 0; k < this.n; k++) {
      const To = mat4Mul(this.F0[k], this.ORIG[k]);
      this.AXES.push({ p: mat4Trans(To), u: unit(mat4Vec(To, this.AXL[k])) });
    }
    this.AXES.push(this.AXES[this.n - 1]);
    const Ttip = mat4Mul(this.F0[this.n], this.TIP);
    const an = this.AXES[this.n];
    this.OUTIL = projDroite(mat4Trans(Ttip), an.p, an.u);
    const a0 = this.AXES[0];
    const O0 = projDroite([0, 0, 0], a0.p, a0.u);
    let x0 = null;
    for (const c of [[1, 0, 0], [0, 1, 0], [0, 0, 1]]) {
      const v = sub(c, mul(a0.u, dot(c, a0.u)));
      if (norm(v) > 0.5) { x0 = unit(v); break; }
    }
    this.REP0 = { O: O0, x: x0, z: a0.u.slice() };
    // visuels (géométries à construire côté affichage)
    this.visuels = [];
    for (const a of attache) {
      const el = urdf.liens[a.lien];
      if (!el) continue;
      for (const vis of enfants(el, 'visual')) {
        const geom = premier(vis, 'geometry');
        if (!geom) continue;
        const g = Array.from(geom.childNodes).find((n) => n.nodeType === 1);
        if (!g) continue;
        const Tt = mat4Mul(a.Tl, enMm(origine(vis)));
        const v = { lien: a.lien, m: a.m, T: Tt, type: g.nodeName };
        if (g.nodeName === 'mesh') { v.fichier = attr(g, 'filename') || ''; v.echelle = vec(attr(g, 'scale'), [1, 1, 1]); }
        else if (g.nodeName === 'box') v.taille = vec(attr(g, 'size'), [0.1, 0.1, 0.1]).map((s) => s * 1000);
        else if (g.nodeName === 'cylinder') { v.rayon = Number(attr(g, 'radius')) * 1000; v.longueur = Number(attr(g, 'length')) * 1000; }
        else if (g.nodeName === 'sphere') v.rayon = Number(attr(g, 'radius')) * 1000;
        else continue;
        this.visuels.push(v);
      }
    }
    this.taille = this._taille();
  }

  fk(qDeg) {
    const F = [I4()];
    for (let k = 0; k < this.n; k++) {
      const R = rotAxe(this.AXL[k], rad(qDeg[k] || 0));
      F.push(mat4Mul(mat4Mul(F[F.length - 1], this.ORIG[k]), mat4From(R)));
    }
    return F;
  }

  _taille() {
    const pts = this.F0.map(mat4Trans).concat([this.OUTIL]);
    this.visuels.forEach((v) => pts.push(mat4Trans(mat4Mul(this.F0[v.m], v.T))));
    const mn = [0, 1, 2].map((i) => Math.min(...pts.map((p) => p[i])));
    const mx = [0, 1, 2].map((i) => Math.max(...pts.map((p) => p[i])));
    return Math.max(norm(sub(mx, mn)), 300);
  }

  // ---------------------------------------------------------------- solution de référence
  solutionReperes() {
    const reps = [{ O: this.REP0.O.slice(), x: this.REP0.x.slice(), z: this.REP0.z.slice() }];
    for (let i = 1; i <= this.n; i++) {
      const { p: zpPt, u: zp } = this.AXES[i - 1];
      const { p: zcPt, u: zc } = this.AXES[i];
      const { O: Op, x: xp } = reps[reps.length - 1];
      let Oi, xi;
      if (i === this.n) { Oi = this.OUTIL.slice(); xi = xp.slice(); }
      else {
        const nrm = cross(zp, zc);
        if (norm(nrm) > TOL_PARALLELE) {
          const [f1, f2] = piedsNormaleCommune(zpPt, zp, zcPt, zc);
          Oi = f2;
          const rho = sub(f2, f1);
          if (norm(rho) > TOL_SECANT) xi = unit(rho);
          else { xi = unit(nrm); if (dot(xi, xp) < 0) xi = neg(xi); }
        } else {
          Oi = projDroite(Op, zcPt, zc);
          const rho = sub(Oi, Op);
          xi = norm(rho) > TOL_SECANT ? unit(rho) : xp.slice();
        }
      }
      reps.push({ O: Oi, x: xi, z: zc.slice() });
    }
    return reps;
  }

  static parametres(rp, r) {
    const w = sub(r.O, rp.O);
    return [deg(angleSigne(rp.x, r.x, rp.z)), dot(w, rp.z), dot(w, r.x), deg(angleSigne(rp.z, r.z, r.x))];
  }

  tableauReference() {
    const R = this.solutionReperes();
    return R.slice(1).map((_, k) => Robot.parametres(R[k], R[k + 1]));
  }

  // ---------------------------------------------------------------- analyse des repères posés
  analyser(reperes, tolPos = 10, tolAng = 6) {
    const reps = [{ O: this.REP0.O.slice(), x: this.REP0.x.slice(), z: this.REP0.z.slice() }];
    const etats = [], lignes = [];
    let bloque = false;
    const f0 = (v) => v.toFixed(0);
    for (let i = 1; i <= this.n; i++) {
      const { p: zpPt, u: zp } = this.AXES[i - 1];
      const { p: zcPt, u: zc } = this.AXES[i];
      const ent = { i, statut: 'todo', msgs: [], params: null };
      etats.push(ent);
      if (bloque) { ent.msgs.push('En attente des repères précédents.'); continue; }
      const rep = (reperes && (reperes[i] || reperes[String(i)])) || {};
      const valide3 = (v) => Array.isArray(v) && v.length === 3 && v.every((a) => typeof a === 'number' && isFinite(a));
      if (!valide3(rep.O) || !valide3(rep.x)) { ent.msgs.push('À compléter : origine O et direction x.'); bloque = true; continue; }
      if (norm(rep.x) < 1e-9) { ent.statut = 'err'; ent.msgs.push('Le vecteur x est nul.'); bloque = true; continue; }
      let okI = true;
      const xRaw = unit(rep.x);
      const dax = distDroite(rep.O, zcPt, zc);
      if (dax > tolPos) {
        ent.msgs.push(`L'origine O_${i} est à ${f0(dax)} mm de l'axe z_${i} (axe de l'articulation ${Math.min(i + 1, this.n)}) : elle doit être SUR cet axe.`);
        okI = false;
      }
      let O = projDroite(rep.O, zcPt, zc);
      const { O: Op, x: xp } = reps[reps.length - 1];
      if (i === this.n) {
        if (norm(sub(O, this.OUTIL)) <= tolPos) O = this.OUTIL.slice();
      } else if (norm(cross(zp, zc)) > TOL_PARALLELE) {
        const [, f2] = piedsNormaleCommune(zpPt, zp, zcPt, zc);
        if (norm(sub(O, f2)) <= tolPos) O = f2;
        else {
          ent.msgs.push(`O_${i} doit être au pied de la normale commune à z_${i - 1} et z_${i} (vous êtes à ${f0(norm(sub(O, f2)))} mm de ce point).`);
          okI = false;
        }
      } else {
        const Oz = projDroite(Op, zcPt, zc);
        if (norm(sub(O, Oz)) <= tolPos) O = Oz;
      }
      const angPerp = deg(Math.asin(Math.min(1, Math.abs(dot(xRaw, zp)))));
      if (angPerp > tolAng) {
        ent.msgs.push(`(DH1) x_${i} n'est pas perpendiculaire à z_${i - 1} (écart ${f0(angPerp)}°).`);
        okI = false;
      }
      let x = sub(xRaw, mul(zp, dot(xRaw, zp)));
      if (norm(x) < 1e-9) { ent.statut = 'err'; ent.msgs.push(`x_${i} est colinéaire à z_${i - 1}.`); bloque = true; continue; }
      x = unit(x);
      const w = sub(O, Op);
      const d = dot(w, zp);
      const rho = sub(w, mul(zp, d));
      const nrho = norm(rho);
      if (i < this.n && nrho > 3.0) {
        const angRho = deg(Math.acos(Math.min(1, Math.abs(dot(unit(rho), x)))));
        if (angRho > tolAng) {
          ent.msgs.push(`(DH2) x_${i} ne rencontre pas z_${i - 1} : le vecteur O_${i - 1}→O_${i} n'est pas dans la direction de x_${i} (écart ${f0(angRho)}°).`);
          okI = false;
        } else {
          x = dot(unit(rho), x) > 0 ? unit(rho) : neg(unit(rho));
          if (Math.abs(dot(rho, zc)) > tolPos) {
            ent.msgs.push(`x_${i} doit aussi être perpendiculaire à z_${i} : c'est la normale COMMUNE à z_${i - 1} et z_${i}.`);
            okI = false;
          }
        }
      } else if (i < this.n) {
        const nrm = cross(zp, zc);
        if (norm(nrm) > TOL_PARALLELE) {
          const angN = deg(Math.acos(Math.min(1, Math.abs(dot(x, unit(nrm))))));
          if (angN > tolAng) {
            ent.msgs.push(`z_${i - 1} et z_${i} sont sécants (en O_${i}) : x_${i} doit être perpendiculaire aux deux axes (écart ${f0(angN)}°).`);
            okI = false;
          } else x = dot(unit(nrm), x) > 0 ? unit(nrm) : neg(unit(nrm));
        }
      }
      if (i === this.n && nrho <= 3.0 && deg(Math.acos(Math.max(-1, Math.min(1, dot(x, xp))))) <= tolAng) x = xp.slice();
      if (i === this.n) {
        const dt = norm(sub(O, this.OUTIL));
        if (dt > tolPos) {
          ent.msgs.push(`O_${i} est à ${f0(dt)} mm de l'origine de l'outil (bride) ; le tableau reste cohérent mais l'outil n'est pas à la bride.`);
          ent.statut = 'warn';
        }
      }
      const r_ = { O, x, z: zc.slice() };
      const p = Robot.parametres({ O: Op, x: xp, z: zp }, r_);
      reps.push(r_);
      lignes.push(p);
      ent.params = p;
      if (okI) {
        ent.statut = ent.statut === 'warn' ? 'warn' : 'ok';
        if (dot(sub(O, Op), x) < -1e-6 && ent.statut === 'ok') ent.msgs.push('r négatif : valide (x_i pointe à l\'opposé de la normale commune) ; vous pouvez inverser x_i pour avoir r > 0.');
      } else ent.statut = 'err';
    }
    const complet = lignes.length === this.n;
    const ok = complet && etats.every((e) => e.statut === 'ok' || e.statut === 'warn');
    return { ok, tableau: lignes, reperes: reps, etats };
  }

  // ---------------------------------------------------------------- construction guidée par curseurs
  construire(params, tolPos = 10, tolAng = 6) {
    params = params || {};
    const L = Math.max(500, 0.9 * this.taille);
    const reperes = {};
    const frames = [];
    let prev = { O: this.REP0.O.slice(), x: this.REP0.x.slice(), z: this.REP0.z.slice() };
    let bloque = false;
    for (let i = 1; i <= this.n; i++) {
      const { p: zpPt, u: zp } = this.AXES[i - 1];
      const { p: zcPt, u: zc } = this.AXES[i];
      const pr = params[i] || params[String(i)] || {};
      const nrm = cross(zp, zc);
      const parallele = norm(nrm) < TOL_PARALLELE;
      const fr = { i, bloque, origine: { mode: 'libre', t: null, tmin: -L, tmax: L, nature: null }, x: { mode: null }, O: null, xv: null, axePt: zcPt.slice(), axeU: zc.slice() };
      frames.push(fr);
      if (bloque) continue;
      const { O: Op, x: xp } = prev;
      let t = null;
      if (i < this.n && !parallele) {
        const [f1, f2] = piedsNormaleCommune(zpPt, zp, zcPt, zc);
        if (norm(sub(f2, f1)) < TOL_SECANT) { t = dot(sub(f2, zcPt), zc); fr.origine = Object.assign(fr.origine, { mode: 'force', t, nature: 'sécants' }); }
      }
      if (t === null) {
        if (pr.t === undefined || pr.t === null) { bloque = true; fr.bloque = false; fr.attenteOrigine = true; continue; }
        t = Number(pr.t);
        fr.origine.t = t;
      }
      const O = add(zcPt, mul(zc, t));
      const w = sub(O, Op);
      const d = dot(w, zp);
      const rho = sub(w, mul(zp, d));
      const nr = norm(rho);
      let base, mode, nat = null;
      if (nr > 3.0) { base = mul(rho, 1 / nr); mode = 'force'; nat = 'normale'; }
      else if (!parallele) { base = unit(nrm); if (dot(base, xp) < 0) base = neg(base); mode = 'force'; nat = 'sécants'; }
      else { base = null; mode = 'libre'; }
      let x;
      if (mode === 'force') {
        const sg = (pr.sign === -1 || pr.sign === '-1') ? -1 : 1;
        x = mul(base, sg);
        fr.x = { mode: 'force', nature: nat, sign: sg };
      } else {
        let e1 = sub(xp, mul(zp, dot(xp, zp)));
        e1 = norm(e1) > 1e-9 ? unit(e1) : unit(cross(zp, [0.3, 0.5, 0.8]));
        const e2 = cross(zp, e1);
        const phi = Number(pr.phi || 0);
        x = add(mul(e1, Math.cos(rad(phi))), mul(e2, Math.sin(rad(phi))));
        fr.x = { mode: 'libre', phi };
      }
      fr.O = O; fr.xv = x;
      reperes[i] = { O, x };
      const res = this.analyser(reperes, tolPos, tolAng);
      if (res.reperes.length > i) prev = res.reperes[i];
      else bloque = true;
    }
    return { frames, analyse: this.analyser(reperes, tolPos, tolAng) };
  }

  parametresSolution() {
    const R = this.solutionReperes();
    const params = {};
    for (let i = 1; i <= this.n; i++) {
      const { p: zcPt, u: zc } = this.AXES[i];
      const zp = this.AXES[i - 1].u;
      params[i] = { t: dot(sub(R[i].O, zcPt), zc), sign: 1, phi: deg(angleSigne(R[i - 1].x, R[i].x, zp)) };
      const xc = this.construire(params).frames[i - 1].xv;
      if (xc && dot(xc, R[i].x) < 0) params[i].sign = -1;
    }
    return params;
  }

  // ---------------------------------------------------------------- aides
  aide(i, niveau, maxNiveau = 3) {
    niveau = Math.min(niveau, maxNiveau);
    if (i < 1 || i > this.n) return `Choisissez un repère entre 1 et ${this.n}.`;
    const R = this.solutionReperes();
    const { p: zpPt, u: zp } = this.AXES[i - 1];
    const { p: zcPt, u: zc } = this.AXES[i];
    const r1 = (v) => v.map((a) => Math.round(a * 10) / 10);
    const r3 = (v) => v.map((a) => Math.round(a * 1000) / 1000);
    const out = [];
    if (i === this.n) {
      out.push(`Repère ${i} (dernier) : z_${i} est confondu avec z_${i - 1} (direction d'approche de l'outil). L'origine O_${i} se place au centre de la bride, sur l'axe de la dernière articulation ; x_${i} est libre (perpendiculaire à z_${i}).`);
      if (niveau >= 3) out.push(`Réponse : O_${i} = [${r1(R[i].O)}] mm ; x_${i} = [${r3(R[i].x)}].`);
      return out.join('\n');
    }
    const nat = natureAxes(zpPt, zp, zcPt, zc);
    out.push(`Repère ${i} : on regarde les axes z_${i - 1} (articulation ${i}) et z_${i} (articulation ${i + 1}).`);
    if (niveau >= 1) out.push(`Niveau 1 — ces deux axes sont ${nat}.`);
    if (niveau >= 2) {
      if (nat.startsWith('parallèles')) out.push(`Niveau 2 — axes parallèles : une infinité de normales communes ; on prend celle qui passe par O_${i - 1} (d_${i} = 0). O_${i} est le projeté de O_${i - 1} sur z_${i} ; x_${i} pointe de O_${i - 1} vers O_${i}.`);
      else if (nat.startsWith('sécants')) out.push(`Niveau 2 — axes sécants : O_${i} est le point d'intersection ; x_${i} est perpendiculaire aux deux axes (produit vectoriel z_${i - 1} × z_${i}), orienté comme x_${i - 1} si possible.`);
      else out.push(`Niveau 2 — axes gauches : x_${i} est porté par la normale commune, orientée de z_${i - 1} vers z_${i} (r_${i} > 0) ; O_${i} est le pied de cette normale sur z_${i}.`);
    }
    if (niveau >= 3) {
      const p = Robot.parametres(R[i - 1], R[i]);
      out.push(`Niveau 3 — O_${i} = [${r1(R[i].O)}] mm ; x_${i} = [${r3(R[i].x)}] → θ0=${p[0].toFixed(2)}° d=${p[1].toFixed(2)} r=${p[2].toFixed(2)} α=${p[3].toFixed(2)}°`);
    }
    return out.join('\n');
  }

  // ---------------------------------------------------------------- validation d'un tableau DH externe
  validerTableau(T, nTests = 15, tolMm = 0.5, tolDeg = 0.1) {
    if (!Array.isArray(T) || T.length !== this.n || T.some((l) => l.length !== 4)) return { ok: false, rapport: [`Le tableau doit avoir ${this.n} lignes de 4 valeurs (θ0, d, r, α).`] };
    // repère 0 imposé (pas toujours confondu avec la base : z0 est l'axe de l'articulation 1, parfois orienté vers le bas)
    const { O: O0, x: x0, z: z0 } = this.REP0, y0 = cross(z0, x0);
    const B = mat4From([x0[0], y0[0], z0[0], x0[1], y0[1], z0[1], x0[2], y0[2], z0[2]], O0);
    const dhFk = (q) => { let M = B.slice(); return T.map((l, k) => (M = mat4Mul(M, matriceDH(l[0] + q[k], l[1], l[2], l[3])))); };
    let graine = 12345;
    const alea = () => { graine = (graine * 1664525 + 1013904223) % 4294967296; return graine / 4294967296; };
    const T0 = dhFk(new Array(this.n).fill(0));
    const C0 = T0.map((t, k) => mat4Mul(mat4InvRigid(t), this.F0[k + 1]));
    const pire = Array.from({ length: this.n }, () => [0, 0]);
    for (let s = 0; s < nTests; s++) {
      const q = this.limites.map(([a, b]) => { const lo = Math.max(a, -180), hi = Math.min(b, 180); return lo + alea() * (hi - lo); });
      const Ts = dhFk(q), F = this.fk(q);
      for (let k = 0; k < this.n; k++) {
        const C = mat4Mul(mat4InvRigid(Ts[k]), F[k + 1]);
        const dp = norm(sub(mat4Trans(C), mat4Trans(C0[k])));
        const tr = [0, 1, 2].reduce((acc, i) => acc + [0, 1, 2].reduce((a2, j) => a2 + C[i * 4 + j] * C0[k][i * 4 + j], 0), 0);
        const da = deg(Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))));
        pire[k] = [Math.max(pire[k][0], dp), Math.max(pire[k][1], da)];
      }
    }
    let ok = true, premiere = null;
    const rapport = pire.map((p, k) => {
      const bon = p[0] <= tolMm && p[1] <= tolDeg;
      ok = ok && bon;
      if (!bon && premiere === null) premiere = k + 1;
      return `${bon ? '✔' : '✘'} 0T${k + 1} : écart max ${p[0].toFixed(2)} mm, ${p[1].toFixed(2)}°`;
    });
    if (ok) rapport.push('→ Tableau DH validé : il reproduit la cinématique de l\'URDF.');
    else if (premiere > 1) rapport.push(`→ Première incohérence sur 0T${premiere} : erreur dans la ligne ${premiere} ou la ligne ${premiere - 1} (α_${premiere - 1} fixe l'axe z_${premiere - 1}).`);
    else rapport.push('→ Incohérence dès 0T1 : vérifiez la ligne 1 (signe des angles).');
    return { ok, rapport };
  }
}

export function natureAxes(p1, u1, p2, u2) {
  if (norm(cross(u1, u2)) < TOL_PARALLELE) return 'parallèles';
  const [f1, f2] = piedsNormaleCommune(p1, u1, p2, u2);
  return norm(sub(f2, f1)) < TOL_SECANT ? 'sécants' : 'non coplanaires (gauches)';
}

export function matriceDH(thetaDeg, d, r, alphaDeg) {
  const t = rad(thetaDeg), a = rad(alphaDeg);
  const ct = Math.cos(t), st = Math.sin(t), ca = Math.cos(a), sa = Math.sin(a);
  return [ct, -st * ca, st * sa, r * ct, st, ct * ca, -ct * sa, r * st, 0, sa, ca, d, 0, 0, 0, 1];
}

// ------------------------------------------------------------------ exports du tableau
const nettoyer = (v) => (Math.abs(v) < 0.005 ? 0 : v); // écarte le bruit numérique (URDF aux angles arrondis)
const fmt = (v, nd = 3) => { let s = nettoyer(v).toFixed(nd); if (s.includes('.')) s = s.replace(/0+$/, '').replace(/\.$/, ''); return s === '-0' || s === '' ? '0' : s; };
export function formaterTableau(T, format, nomRobot = 'robot') {
  if (format === 'csv') return 'i,theta0_deg,d_mm,r_mm,alpha_deg\n' + T.map((l, i) => `${i + 1},${fmt(l[0])},${fmt(l[1])},${fmt(l[2])},${fmt(l[3])}`).join('\n');
  if (format === 'json') {
    return JSON.stringify({
      robot: nomRobot, convention: 'DH standard', unites: { longueur: 'mm', angle: 'deg' },
      lignes: T.map((l, i) => ({ i: i + 1, theta0: +nettoyer(l[0]).toFixed(3), d: +nettoyer(l[1]).toFixed(3), r: +nettoyer(l[2]).toFixed(3), alpha: +nettoyer(l[3]).toFixed(3) })),
    }, null, 2);
  }
  if (format === 'tex') {
    const lg = T.map((l, i) => `${i + 1} & ${fmt(l[0], 2)} & ${fmt(l[1], 2)} & ${fmt(l[2], 2)} & ${fmt(l[3], 2)} \\\\`).join('\n');
    return '\\begin{tabular}{c|cccc}\n$i$ & $\\theta_{0,i}$ (°) & $d_i$ (mm) & $r_i$ (mm) & $\\alpha_i$ (°) \\\\ \\hline\n' + lg + '\n\\end{tabular}';
  }
  const out = [`Tableau DH standard — ${nomRobot}`, ' i |  theta0 (°) |    d (mm) |    r (mm) |  alpha (°)', '---+-------------+-----------+-----------+-----------'];
  T.forEach((l, i) => out.push(`${String(i + 1).padStart(2)} | ${l[0].toFixed(3).padStart(11)} | ${l[1].toFixed(3).padStart(9)} | ${l[2].toFixed(3).padStart(9)} | ${l[3].toFixed(3).padStart(10)}`));
  return out.join('\n');
}
