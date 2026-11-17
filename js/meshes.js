// meshes.js — lecture des maillages (STL, OBJ, DAE, GLB/GLTF) et primitives URDF ; géométries renvoyées en mm, dans le repère du lien.
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { ext } from './files.js';

// Fusionne tous les maillages d'un objet 3D en une seule BufferGeometry (positions + normales), en appliquant les transformations de la scène.
function fusionner(racine, pre = null) {
  racine.updateMatrixWorld(true);
  const pos = [];
  racine.traverse((o) => {
    if (!o.isMesh || !o.geometry) return;
    let g = o.geometry.index ? o.geometry.toNonIndexed() : o.geometry.clone();
    const M = o.matrixWorld.clone();
    if (pre) M.premultiply(pre);
    g.applyMatrix4(M);
    const a = g.getAttribute('position');
    for (let i = 0; i < a.count; i++) pos.push(a.getX(i), a.getY(i), a.getZ(i));
  });
  if (!pos.length) throw new Error('maillage vide');
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.computeVertexNormals();
  return g;
}

export async function chargerGeometrie(entree, echelle = [1, 1, 1]) {
  const e = ext(entree.chemin);
  const tampon = await entree.lire();
  let g;
  if (e === 'stl') {
    g = new STLLoader().parse(tampon);
    g.deleteAttribute('color');
    if (!g.getAttribute('normal')) g.computeVertexNormals();
  } else if (e === 'obj') {
    g = fusionner(new OBJLoader().parse(new TextDecoder().decode(tampon)));
  } else if (e === 'dae') {
    let txt = new TextDecoder().decode(tampon).replace(/<library_images>[\s\S]*?<\/library_images>/g, '');
    const avert = console.warn; console.warn = () => {};          // le chargeur signale chaque asset Z_UP : information inutile ici
    let res; try { res = new ColladaLoader().parse(txt, ''); } finally { console.warn = avert; }
    // Le chargeur three.js convertit Z_UP en Y_UP (rotation -90° autour de X) ; la convention ROS est Z vers le haut :
    // dans les deux cas (Z_UP déjà annulé, Y_UP converti) il faut appliquer +90° autour de X sur la scène chargée.
    g = fusionner(res.scene, new THREE.Matrix4().makeRotationX(Math.PI / 2));
  } else if (e === 'glb' || e === 'gltf') {
    const gltf = await new GLTFLoader().parseAsync(e === 'gltf' ? new TextDecoder().decode(tampon) : tampon, '');
    g = fusionner(gltf.scene);
  } else throw new Error(`format non géré : .${e}`);
  g.scale(echelle[0] * 1000, echelle[1] * 1000, echelle[2] * 1000);   // URDF en mètres → mm
  g.computeBoundingSphere();
  return g;
}

// Primitives URDF (dimensions déjà en mm). Le cylindre URDF a son axe selon z.
export function geometriePrimitive(v) {
  if (v.type === 'box') return new THREE.BoxGeometry(...v.taille);
  if (v.type === 'sphere') return new THREE.SphereGeometry(v.rayon, 32, 16);
  if (v.type === 'cylinder') { const g = new THREE.CylinderGeometry(v.rayon, v.rayon, v.longueur, 40); g.rotateX(Math.PI / 2); return g; }
  return null;
}
