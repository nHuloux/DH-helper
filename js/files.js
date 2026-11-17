// files.js — accès aux fichiers du robot (dossier déposé, archive .zip, exemples hébergés). Tout reste dans le navigateur.
const MAX_FICHIERS = 8000;

const normaliser = (p) => {
  const sortie = [];
  for (const s of p.replace(/\\/g, '/').split('/')) {
    if (!s || s === '.') continue;
    if (s === '..') { sortie.pop(); continue; }
    sortie.push(s);
  }
  return sortie.join('/');
};
const dossierDe = (p) => p.split('/').slice(0, -1).join('/');
const ext = (p) => (p.includes('.') ? p.split('.').pop().toLowerCase() : '');

export class FileStore {
  constructor() { this.fichiers = new Map(); this._index = null; }
  ajouter(chemin, lire, taille = 0) {
    const c = normaliser(chemin);
    if (!c || c.split('/').some((s) => s.startsWith('.') && s !== '.')) { if (!c || /(^|\/)(__MACOSX|\.git|\.DS_Store)/.test(c) || c.split('/').pop().startsWith('._')) return; }
    if (this.fichiers.size >= MAX_FICHIERS) throw new Error(`Trop de fichiers (plus de ${MAX_FICHIERS}).`);
    this.fichiers.set(c, { chemin: c, nom: c.split('/').pop(), taille, lire });
    this._index = null;
  }
  get taille() { return this.fichiers.size; }
  parExtension(e) { return [...this.fichiers.values()].filter((f) => ext(f.chemin) === e).sort((a, b) => a.chemin.length - b.chemin.length || a.chemin.localeCompare(b.chemin)); }
  urdfs() { return this.parExtension('urdf'); }
  xacros() { return this.fichiers.size ? [...this.fichiers.values()].filter((f) => /\.(xacro|urdf\.xacro)$/i.test(f.chemin)) : []; }
  async texte(entree) { return new TextDecoder('utf-8').decode(await entree.lire()); }

  // Retrouve un fichier à partir du nom indiqué dans l'URDF : relatif, package://, file://, ou simple suffixe de chemin.
  trouver(nomFichier, dossierUrdf = '') {
    if (!nomFichier) return null;
    if (!this._index) { this._index = new Map(); for (const f of this.fichiers.values()) this._index.set(f.chemin.toLowerCase(), f); }
    let s = nomFichier.trim().replace(/^(package|file|model):\/\//i, '');
    s = decodeURIComponent(s.replace(/\\/g, '/'));
    const candidats = [normaliser(dossierUrdf ? `${dossierUrdf}/${s}` : s), normaliser(s)];
    const parties = normaliser(s).split('/');
    for (const c of candidats) { const f = this._index.get(c.toLowerCase()); if (f) return f; }
    for (let k = 0; k < parties.length; k++) {                // suffixes de plus en plus courts (le nom de paquet disparaît le premier)
      const suf = parties.slice(k).join('/').toLowerCase();
      const trouves = [...this._index.entries()].filter(([c]) => c === suf || c.endsWith(`/${suf}`)).map(([, f]) => f);
      if (trouves.length) {
        const ref = dossierUrdf.toLowerCase();
        const commun = (c) => { let n = 0; const a = c.toLowerCase().split('/'), b = ref.split('/'); while (n < a.length && n < b.length && a[n] === b[n]) n++; return n; };
        trouves.sort((a, b) => commun(b.chemin) - commun(a.chemin) || a.chemin.length - b.chemin.length);
        return trouves[0];
      }
    }
    return null;
  }
}
export { dossierDe, normaliser, ext };

// ---------------------------------------------------------------- sources
export async function ajouterZip(store, fichierZip) {
  if (!window.JSZip) throw new Error('Bibliothèque JSZip non chargée.');
  const zip = await window.JSZip.loadAsync(fichierZip);
  const noms = Object.keys(zip.files).filter((n) => !zip.files[n].dir);
  if (noms.length > MAX_FICHIERS) throw new Error(`Archive trop volumineuse (${noms.length} fichiers).`);
  for (const n of noms) store.ajouter(n, () => zip.files[n].async('arraybuffer'));
}

export async function depuisListe(fichiers) {
  const store = new FileStore();
  for (const f of fichiers) {
    if (ext(f.name) === 'zip') await ajouterZip(store, f);
    else store.ajouter(f.webkitRelativePath || f.name, () => f.arrayBuffer(), f.size);
  }
  return store;
}

function lireEntrees(lecteur) {
  return new Promise((ok, ko) => {
    const tout = [];
    const suite = () => lecteur.readEntries((lot) => { if (!lot.length) ok(tout); else { tout.push(...lot); suite(); } }, ko);
    suite();
  });
}
async function parcourir(entree, chemin, store, zips) {
  if (entree.isFile) {
    const f = await new Promise((ok, ko) => entree.file(ok, ko));
    if (ext(f.name) === 'zip') zips.push(f); else store.ajouter(`${chemin}${f.name}`, () => f.arrayBuffer(), f.size);
  } else if (entree.isDirectory) {
    for (const e of await lireEntrees(entree.createReader())) await parcourir(e, `${chemin}${entree.name}/`, store, zips);
  }
}
// Dépôt (glisser-déposer) : les entrées doivent être extraites AVANT tout await.
export async function depuisDepot(dataTransfer) {
  const entrees = [], fichiers = [];
  for (const it of dataTransfer.items || []) {
    if (it.kind !== 'file') continue;
    const e = it.webkitGetAsEntry ? it.webkitGetAsEntry() : null;
    if (e) entrees.push(e); else if (it.getAsFile()) fichiers.push(it.getAsFile());
  }
  if (!entrees.length && !fichiers.length) fichiers.push(...(dataTransfer.files || []));
  const store = new FileStore(), zips = [];
  for (const e of entrees) await parcourir(e, '', store, zips);
  for (const f of fichiers) { if (ext(f.name) === 'zip') zips.push(f); else store.ajouter(f.name, () => f.arrayBuffer(), f.size); }
  for (const z of zips) await ajouterZip(store, z);
  return store;
}

// Exemples hébergés avec le site : robots/index.json donne la liste des fichiers de chaque robot.
export function depuisExemple(base, fichiers) {
  const store = new FileStore();
  for (const f of fichiers) store.ajouter(f, async () => {
    const r = await fetch(`${base}/${f}`);
    if (!r.ok) throw new Error(`${f} : HTTP ${r.status}`);
    return r.arrayBuffer();
  });
  return store;
}
