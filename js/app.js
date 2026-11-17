// app.js — interface : chargement d'un URDF (exemples / dossier / zip), placement guidé des repères DH, tableau, exports.
import { Viewer } from './viewer.js';
import { lireURDF, Robot, formaterTableau } from './robot.js';
import { depuisListe, depuisDepot, depuisExemple, dossierDe } from './files.js';
import { chargerGeometrie } from './meshes.js';
import { argumentsXacro, xacroVersUrdf } from './xacro.js';

const $ = (id) => document.getElementById(id);
const parametres = new URLSearchParams(location.search);
const AIDE_MAX = parametres.has('aide') ? Math.max(0, Math.min(3, parseInt(parametres.get('aide'), 10) || 0)) : 3;
const PROF = parametres.get('prof') === '1';
const f2 = (v) => Math.round(v * 100) / 100;
const echapper = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ------------------------------------------------------------------ état
const E = {
  store: null, entree: null, robot: null, urdf: null, cache: new Map(),
  P: {}, cur: 1, q: [], F: null, analyse: null, meta: null, minuteur: null, exemples: [],
};
const viewer = new Viewer($('vue'), {
  auSurvol: (p) => { const e = $('pick'); if (!p) { e.style.display = 'none'; return; } e.style.display = 'block'; e.textContent = `x = ${p[0].toFixed(1)}   y = ${p[1].toFixed(1)}   z = ${p[2].toFixed(1)} mm`; },
});

let tToast = null;
function toast(msg, ms = 4500) {
  const t = $('toast'); t.textContent = msg; t.style.display = 'block';
  clearTimeout(tToast); tToast = setTimeout(() => { t.style.display = 'none'; }, ms);
}

// ------------------------------------------------------------------ fenêtre de chargement
function ouvrirDlg(message = '') {
  $('dlg').style.display = 'flex'; $('choixurdf').style.display = 'none'; $('xacroForm').style.display = 'none';
  $('bFermer').style.display = E.robot ? '' : 'none';
  setMsg(message);
}
function fermerDlg() { $('dlg').style.display = 'none'; }
function setMsg(texte, erreur = false) { const m = $('msgdlg'); m.textContent = texte; m.className = 'msgdlg' + (erreur ? ' err' : ''); }

async function ouvrirStore(store) {
  setMsg(''); $('xacroForm').style.display = 'none';
  if (!store.taille) { setMsg('Aucun fichier reçu.', true); return; }
  const urdfs = store.urdfs();
  // xacro : ceux qui déclarent des arguments et une racine <robot> sont des fichiers « principaux » ; les autres sont des morceaux inclus
  const xacros = await Promise.all(store.xacros().map(async (f) => {
    let principal = false;
    try { if (f.taille < 400000) { const t = await store.texte(f); principal = /<robot\b/.test(t) && !/<xacro:macro\b/.test(t.slice(0, 1500)) && (/<xacro:arg\b/.test(t) || /<link\b/.test(t) || /<xacro:include\b/.test(t)); } } catch (e) { principal = false; }
    return Object.assign(f, { principal });
  }));
  xacros.sort((x, y) => Number(y.principal) - Number(x.principal) || Number(/\.urdf\.xacro$/i.test(y.chemin)) - Number(/\.urdf\.xacro$/i.test(x.chemin)) || x.chemin.localeCompare(y.chemin));
  const candidats = [...urdfs, ...xacros];
  if (!candidats.length) { setMsg('Aucun fichier .urdf ni .xacro trouvé dans ce dépôt.', true); return; }
  if (candidats.length === 1) { await ouvrirCandidat(store, candidats[0]); return; }
  const z = $('choixurdf'); z.style.display = 'block';
  z.innerHTML = `<h3>Plusieurs fichiers de description trouvés — lequel ouvrir ?</h3>${xacros.length ? '<div class="hint">Pour un xacro, choisissez le fichier <b>principal</b> (celui qui assemble le robot, souvent <code>*.urdf.xacro</code>) ; les fichiers « inclus » ne sont que des morceaux de modèle.</div>' : ''}`;
  candidats.forEach((u) => { const b = document.createElement('button'); b.textContent = u.chemin + (estXacro(u) ? (u.principal ? '   (xacro · principal)' : '   (xacro · inclus)') : ''); b.onclick = () => ouvrirCandidat(store, u); z.appendChild(b); });
}
const estXacro = (e) => /\.xacro$/i.test(e.chemin);
async function ouvrirCandidat(store, entree) {
  if (!estXacro(entree)) { await charger(store, entree); return; }
  try { await preparerXacro(store, entree); } catch (err) { setMsg(err.message || String(err), true); }
}

// ------------------------------------------------------------------ xacro : formulaire d'arguments puis conversion
const cleXacro = (entree) => `dh3x:${entree.chemin}`;
async function preparerXacro(store, entree) {
  const texte = await store.texte(entree);
  const args = argumentsXacro(texte);
  let sauve = {};
  try { sauve = JSON.parse(localStorage.getItem(cleXacro(entree)) || '{}'); } catch (e) { sauve = {}; }
  $('choixurdf').style.display = 'none'; $('xacroForm').style.display = 'block';
  $('xacroNom').textContent = entree.chemin; $('xacroNom').style.textTransform = 'none';
  const g = $('xacroArgs'); g.innerHTML = '';
  if (!args.length) g.innerHTML = '<span class="hint">Ce fichier ne déclare aucun argument.</span>';
  args.forEach((a) => {
    const l = document.createElement('label'); l.textContent = a.nom; l.htmlFor = 'xa_' + a.nom;
    const i = document.createElement('input'); i.id = 'xa_' + a.nom; i.dataset.nom = a.nom; i.value = sauve[a.nom] !== undefined ? sauve[a.nom] : a.defaut;
    g.append(l, i);
  });
  $('bXacro').onclick = () => convertirEtOuvrir(store, entree, texte);
  setMsg('');
}
async function convertirEtOuvrir(store, entree, texte) {
  const args = {};
  document.querySelectorAll('#xacroArgs input').forEach((i) => { args[i.dataset.nom] = i.value; });
  try { localStorage.setItem(cleXacro(entree), JSON.stringify(args)); } catch (e) { /* stockage indisponible */ }
  try {
    setMsg('Conversion du xacro…');
    await store.lirePaquets();
    const xml = await xacroVersUrdf({
      texte, dossier: dossierDe(entree.chemin), args,
      lire: (c) => store.texteChemin(c), paquet: (n) => store.dossierPaquet(n),
    });
    await charger(store, entree, null, xml);
  } catch (err) {
    console.error(err);
    const brut = String(err.message || err).replace(/\s+/g, ' ');
    setMsg(`Conversion impossible : ${brut.slice(0, 300)}. Astuce : si le xacro utilise des fonctions avancées, convertissez-le avec la commande xacro (pip install xacro) puis déposez l'URDF obtenu.`, true);
  }
}

// ------------------------------------------------------------------ chargement d'un robot
async function charger(store, entree, tip = null, xml = null) {
  try {
    setMsg('Lecture de l\'URDF…');
    const texte = xml !== null ? xml : await store.texte(entree);
    if (xml === null && (/xmlns:xacro/.test(texte.slice(0, 5000)) || /<xacro:/.test(texte) || /\$\{/.test(texte))) { await preparerXacro(store, entree); return; }
    const doc = new DOMParser().parseFromString(texte, 'text/xml');
    if (doc.getElementsByTagName('parsererror').length) { const m = /error on line (\d+) at column (\d+): ([^\n]*?)(?:Below is|$)/i.exec(doc.getElementsByTagName('parsererror')[0].textContent || ''); throw new Error('Fichier XML invalide' + (m ? ` (ligne ${m[1]}, colonne ${m[2]} : ${m[3].trim()})` : '') + '.'); }
    const urdf = lireURDF(doc);
    const robot = new Robot(urdf, { tip });
    if (store !== E.store) E.cache = new Map();
    Object.assign(E, { store, entree, urdf, robot, xml });
    $('chargement').style.display = 'flex';
    fermerDlg();
    const { geoms, manquants, erreurs } = await chargerMaillages(robot, store, dossierDe(entree.chemin));
    $('chargement').style.display = 'none';
    viewer.setRobot(robot, geoms);
    initialiserInterface();
    if (manquants.length || erreurs.length) {
      const liste = [...manquants.map((m) => `${m} (introuvable)`), ...erreurs].slice(0, 4).join(' ; ');
      toast(`Certains maillages n'ont pas pu être affichés : ${liste}${manquants.length + erreurs.length > 4 ? '…' : ''}. Les calculs DH ne sont pas affectés.`, 9000);
    }
  } catch (err) {
    console.error(err);
    $('chargement').style.display = 'none';
    ouvrirDlg();
    setMsg(err.message || String(err), true);
  }
}

async function chargerMaillages(robot, store, dossierUrdf) {
  const geoms = new Array(robot.visuels.length).fill(null), manquants = [], erreurs = [];
  let fait = 0;
  const nb = robot.visuels.filter((v) => v.type === 'mesh').length;
  await Promise.all(robot.visuels.map(async (v, i) => {
    if (v.type !== 'mesh') return;
    const f = store.trouver(v.fichier, dossierUrdf);
    if (!f) { manquants.push(v.fichier); fait++; return; }
    const cle = `${f.chemin}|${v.echelle.join(',')}`;
    try {
      if (!E.cache.has(cle)) E.cache.set(cle, chargerGeometrie(f, v.echelle));
      geoms[i] = await E.cache.get(cle);
    } catch (err) { E.cache.delete(cle); erreurs.push(`${f.nom} : ${err.message}`); }
    fait++;
    $('chargement').textContent = `Chargement des maillages… ${fait}/${nb}`;
  }));
  return { geoms, manquants, erreurs };
}

// ------------------------------------------------------------------ interface liée au robot
const cleStockage = () => `dh3:${E.robot.nom}:${E.robot.tip}:${E.robot.n}`;
function sauver() { try { localStorage.setItem(cleStockage(), JSON.stringify(E.P)); } catch (e) { /* stockage indisponible */ } }
function restaurer() { E.P = {}; try { const s = localStorage.getItem(cleStockage()); if (s) E.P = JSON.parse(s) || {}; } catch (e) { E.P = {}; } }

function initialiserInterface() {
  const R = E.robot;
  $('nom').textContent = '— ' + R.nom;
  document.title = `Atelier DH — ${R.nom}`;
  E.cur = 1; E.q = new Array(R.n).fill(0); E.F = R.F0.map((m) => m.slice());
  // branches
  const bons = R.infoBranches.branches.filter((b) => b.valide);
  $('branche').style.display = bons.length > 1 ? 'flex' : 'none';
  const sel = $('tip'); sel.innerHTML = '';
  bons.forEach((b) => { const o = document.createElement('option'); o.value = b.tip; o.textContent = `${b.tip} (${b.nRot} rotoïdes)`; o.selected = b.tip === R.tip; sel.appendChild(o); });
  const r0 = R.REP0;
  $('rep0').innerHTML = `Repère 0 imposé : O₀ = (${r0.O.map(f2).join(', ')}) mm · x₀ = (${r0.x.map(f2).join(', ')}) · z₀ = axe de l'articulation 1 · ${R.n} articulations`;
  const fb = $('fbar'); fb.innerHTML = '';
  for (let i = 1; i <= R.n; i++) { const b = document.createElement('button'); b.className = 'f' + (i === 1 ? ' cur' : ''); b.textContent = i; b.dataset.i = i; b.onclick = () => choisir(i); fb.appendChild(b); }
  const ab = $('abar'); ab.innerHTML = ''; $('aide').style.display = 'none';
  $('secAide').style.display = AIDE_MAX > 0 ? '' : 'none';
  for (let l = 1; l <= AIDE_MAX; l++) { const b = document.createElement('button'); b.textContent = 'Aide niveau ' + l; b.onclick = () => { const e = $('aide'); e.style.display = 'block'; e.textContent = R.aide(E.cur, l, AIDE_MAX); }; ab.appendChild(b); }
  if (AIDE_MAX < 3 && AIDE_MAX > 0) { const s = document.createElement('span'); s.className = 'hint'; s.textContent = ' (la réponse complète est désactivée)'; ab.appendChild(s); }
  const sl = $('sliders'); sl.innerHTML = '';
  R.limites.forEach(([lo, hi], k) => {
    const d = document.createElement('div'); d.className = 'row';
    d.innerHTML = `<label>q${k + 1}</label><input type="range" min="${Math.max(lo, -180)}" max="${Math.min(hi, 180)}" step="1" value="0" data-k="${k}" aria-label="Articulation ${k + 1}"><span style="width:44px;text-align:right" id="qv${k}">0°</span>`;
    sl.appendChild(d);
  });
  sl.querySelectorAll('input').forEach((inp) => {
    inp.oninput = () => { const k = +inp.dataset.k; E.q[k] = +inp.value; $(`qv${k}`).textContent = `${E.q[k]}°`; E.F = R.fk(E.q); viewer.appliquerPose(E.F); };
  });
  $('rapport').style.display = 'none'; $('tabEtu').value = '';
  $('prof').style.display = PROF ? 'block' : 'none';
  restaurer();
  viewer.construireAxes(E.cur);
  construire();
}

function planifier() { clearTimeout(E.minuteur); E.minuteur = setTimeout(construire, 30); }
const tol = () => [parseFloat($('tpos').value) || 10, parseFloat($('tang').value) || 6];
function construire() {
  if (!E.robot) return;
  const [tp, ta] = tol();
  const res = E.robot.construire(E.P, tp, ta);
  E.meta = res.frames; E.analyse = res.analyse; sauver();
  rendreEdition(); rendreResultats();
  viewer.setReperes(E.analyse.reperes, E.cur);
}
function choisir(i) {
  E.cur = i;
  viewer.construireAxes(i);
  if (E.meta) { rendreEdition(); viewer.setReperes(E.analyse.reperes, E.cur); }
}
function majP(champ, val) { E.P[E.cur] = Object.assign({}, E.P[E.cur] || {}, { [champ]: val }); planifier(); }
const subs = (c, t) => document.querySelectorAll('.' + c).forEach((e) => { e.textContent = t; });

function rendreEdition() {
  const cur = E.cur, fr = E.meta[cur - 1], p = E.P[cur] || {};
  $('ti').textContent = cur; $('ti2').textContent = cur; $('tO').textContent = 'O' + cur; $('tX').textContent = '(x' + cur + ')';
  subs('xi', cur); subs('xp', cur - 1); subs('zp', cur - 1);
  const o = fr.origine;
  if (o.mode === 'force') {
    $('oForce').style.display = 'block'; $('oLibre').style.display = 'none';
    $('oForce').innerHTML = `<b>O${cur} est imposé</b> : les axes z${cur - 1} et z${cur} sont sécants, l'origine est leur point d'intersection.`;
  } else {
    $('oForce').style.display = 'none'; $('oLibre').style.display = 'block';
    const ts = $('ts'), tn = $('tn');
    ts.min = Math.round(o.tmin); ts.max = Math.round(o.tmax);
    const v = p.t !== undefined && p.t !== null ? p.t : 0;
    if (document.activeElement !== ts) ts.value = v;
    if (document.activeElement !== tn) tn.value = Math.round(v * 10) / 10;
  }
  $('oCoord').textContent = fr.O ? `O${cur} = (${fr.O.map(f2).join(', ')}) mm` : (o.mode === 'force' ? '' : `O${cur} : pas encore placé (déplacez le curseur).`);
  const x = fr.x || {};
  $('xAttente').style.display = $('xForce').style.display = $('xLibre').style.display = 'none';
  if (fr.bloque || fr.O === null) {
    $('xAttente').style.display = 'block';
    $('xAttente').textContent = fr.bloque ? 'En attente des repères précédents.' : `Placez d'abord l'origine O${cur}.`;
  } else if (x.mode === 'force') {
    $('xForce').style.display = 'block';
    $('xForceTxt').innerHTML = `<b>x${cur} est imposé</b> (au sens près) : ` + (x.nature === 'sécants'
      ? `perpendiculaire à z${cur - 1} et à z${cur} (axes sécants).`
      : `il doit couper z${cur - 1} et rester perpendiculaire à z${cur - 1} : c'est la direction qui joint z${cur - 1} à O${cur}.`) + (x.sign < 0 ? ' <i>(sens inversé)</i>' : '');
  } else if (x.mode === 'libre') {
    $('xLibre').style.display = 'block';
    const ps = $('ps'), pn = $('pn'), v = x.phi || 0;
    if (document.activeElement !== ps) ps.value = v;
    if (document.activeElement !== pn) pn.value = Math.round(v * 10) / 10;
  }
  document.querySelectorAll('#fbar button').forEach((b) => b.classList.toggle('cur', +b.dataset.i === cur));
}

function rendreResultats() {
  const d = E.analyse, R = E.robot;
  const lg = $('log'); lg.innerHTML = '';
  d.etats.forEach((e) => {
    const sym = { ok: '✔', warn: '⚠', err: '✘', todo: '…' }[e.statut];
    let h = `<b>${sym} Repère ${e.i}</b>`;
    if (e.params && (e.statut === 'ok' || e.statut === 'warn')) h += ` → θ₀=${f2(e.params[0])}° d=${f2(e.params[1])} r=${f2(e.params[2])} α=${f2(e.params[3])}°`;
    const div = document.createElement('div'); div.className = e.statut;
    div.innerHTML = h + e.msgs.map((m) => `<div style="margin-left:14px;font-weight:normal">${echapper(m)}</div>`).join('');
    lg.appendChild(div);
  });
  if (d.ok) { const div = document.createElement('div'); div.className = 'ok'; div.innerHTML = '<b>Tous les repères respectent les règles DH.</b>'; lg.appendChild(div); }
  const tb = document.querySelector('#tab tbody'); tb.innerHTML = '';
  for (let i = 0; i < R.n; i++) {
    const p = d.etats[i].params, ok = d.etats[i].statut === 'ok' || d.etats[i].statut === 'warn';
    tb.innerHTML += `<tr><td>${i + 1}</td>${p && ok ? p.map((v) => `<td>${f2(v)}</td>`).join('') : '<td colspan="4" style="text-align:center;color:#aaa">—</td>'}</tr>`;
  }
  d.etats.forEach((e) => {
    const b = document.querySelector(`#fbar button[data-i="${e.i}"]`);
    if (b) b.className = 'f ' + (e.statut === 'todo' ? '' : e.statut) + (e.i === E.cur ? ' cur' : '');
  });
}

// ------------------------------------------------------------------ exports
function telecharger(nom, contenu, type = 'text/plain') {
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([contenu], { type })); a.download = nom; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
function tableauExportable() {
  if (!E.analyse || !E.analyse.ok) { toast('Complétez d\'abord tous les repères (sans erreur) pour exporter le tableau.'); return null; }
  return E.analyse.tableau;
}
const nomFichier = (ext) => `tableau_DH_${E.robot.nom.replace(/[^\w-]+/g, '_')}.${ext}`;

// ------------------------------------------------------------------ liaisons des commandes
function lier() {
  $('ts').oninput = (e) => { $('tn').value = Math.round(+e.target.value * 10) / 10; majP('t', +e.target.value); };
  $('tn').oninput = (e) => { if (e.target.value === '') return; $('ts').value = e.target.value; majP('t', +e.target.value); };
  $('ps').oninput = (e) => { $('pn').value = e.target.value; majP('phi', +e.target.value); };
  $('pn').oninput = (e) => { if (e.target.value === '') return; $('ps').value = e.target.value; majP('phi', +e.target.value); };
  $('xSign').onclick = () => majP('sign', E.P[E.cur] && E.P[E.cur].sign === -1 ? 1 : -1);
  $('vide').onclick = () => { delete E.P[E.cur]; planifier(); };
  $('videTout').onclick = () => { E.P = {}; planifier(); };
  $('bAxes').onclick = (e) => { const v = !viewer.afficherAxes; viewer.montrerAxes(v); e.target.classList.toggle('on', v); };
  $('bFr').onclick = (e) => { const v = !viewer.afficherReperes; viewer.montrerReperes(v); e.target.classList.toggle('on', v); };
  $('bVue').onclick = () => viewer.cadrer();
  $('opac').oninput = (e) => viewer.setOpacite(+e.target.value);
  $('bPng').onclick = async () => { const b = await viewer.png(); telecharger('robot_reperes_DH.png', b, 'image/png'); };
  $('tpos').onchange = $('tang').onchange = planifier;
  $('bZero').onclick = () => { E.q = new Array(E.robot.n).fill(0); document.querySelectorAll('#sliders input').forEach((i) => { i.value = 0; $(`qv${i.dataset.k}`).textContent = '0°'; }); E.F = E.robot.F0.map((m) => m.slice()); viewer.appliquerPose(E.F); };
  $('bSol').onclick = () => { E.P = E.robot.parametresSolution(); construire(); };
  $('tip').onchange = (e) => charger(E.store, E.entree, e.target.value, E.xml);
  document.querySelectorAll('[data-ex]').forEach((b) => {
    b.onclick = () => { const T = tableauExportable(); if (!T) return; const f = b.dataset.ex; telecharger(nomFichier(f), formaterTableau(T, f, E.robot.nom), f === 'json' ? 'application/json' : 'text/plain'); };
  });
  $('cp').onclick = async () => {
    const T = tableauExportable(); if (!T) return;
    const txt = 'i\tθ0 (°)\td (mm)\tr (mm)\tα (°)\n' + T.map((l, i) => `${i + 1}\t${l.map((v) => f2(v)).join('\t')}`).join('\n');
    try { await navigator.clipboard.writeText(txt); $('cp').textContent = 'Copié ✔'; } catch (e) { window.prompt('Copiez le tableau :', txt); }
    setTimeout(() => { $('cp').textContent = 'Copier'; }, 1500);
  };
  $('bVerif').onclick = () => {
    const lignes = $('tabEtu').value.split(/\n+/).map((l) => l.trim()).filter(Boolean);
    const T = lignes.map((l) => l.replace(/(\d),(\d)/g, '$1.$2').split(/[\s;\t]+/).map(Number));
    const r = $('rapport'); r.style.display = 'block';
    const n = E.robot.n;
    const T4 = T.map((l) => (l.length === 5 ? l.slice(1) : l));
    if (T4.length !== n || T4.some((l) => l.length !== 4 || l.some((v) => !isFinite(v)))) { r.textContent = `Le tableau doit contenir ${n} lignes de 4 nombres (θ₀, d, r, α) ; ${T4.length} ligne(s) reconnue(s).`; return; }
    const res = E.robot.validerTableau(T4);
    r.textContent = res.rapport.join('\n');
    r.className = 'pre ' + (res.ok ? 'ok' : 'err');
  };
  // fenêtre de chargement
  $('bAutre').onclick = () => ouvrirDlg();
  $('bFermer').onclick = fermerDlg;
  $('bDossier').onclick = () => $('inDossier').click();
  $('bFichiers').onclick = () => $('inFichiers').click();
  const apresChoix = async (ev) => { const fl = [...ev.target.files]; ev.target.value = ''; try { setMsg('Lecture des fichiers…'); await ouvrirStore(await depuisListe(fl)); } catch (err) { setMsg(err.message, true); } };
  $('inDossier').onchange = apresChoix; $('inFichiers').onchange = apresChoix;
  window.addEventListener('dragover', (e) => { e.preventDefault(); $('depot').classList.add('sur'); if ($('dlg').style.display !== 'flex') ouvrirDlg(); });
  window.addEventListener('dragleave', (e) => { if (!e.relatedTarget) $('depot').classList.remove('sur'); });
  window.addEventListener('drop', async (e) => {
    e.preventDefault(); $('depot').classList.remove('sur');
    try { setMsg('Lecture des fichiers…'); await ouvrirStore(await depuisDepot(e.dataTransfer)); } catch (err) { setMsg(err.message, true); }
  });
  window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && E.robot) fermerDlg(); });
}

async function chargerExemple(ex) {
  try { setMsg(`Chargement de ${ex.nom}…`); await ouvrirStore(depuisExemple(`robots/${ex.dossier}`, ex.fichiers)); } catch (err) { setMsg(err.message, true); }
}

async function demarrer() {
  lier();
  try {
    const r = await fetch('robots/index.json');
    if (r.ok) E.exemples = await r.json();
  } catch (e) { /* hors ligne : pas d'exemples */ }
  const z = $('exemples');
  E.exemples.forEach((ex) => {
    const b = document.createElement('button'); b.innerHTML = `<b>${echapper(ex.nom)}</b><span>${echapper(ex.description || '')}</span>`;
    b.onclick = () => chargerExemple(ex); z.appendChild(b);
  });
  const voulu = parametres.get('robot');
  const ex = voulu && E.exemples.find((e) => e.id === voulu);
  if (ex) chargerExemple(ex); else ouvrirDlg();
}
demarrer();
window.__dh = { E, viewer, construire, choisir, charger, chargerExemple };
