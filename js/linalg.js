// linalg.js — petites fonctions d'algèbre linéaire (vecteurs = tableaux [x,y,z], matrices 4x4 = tableaux de 16, ligne par ligne)
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const unit = (a) => {
  const n = norm(a);
  if (n < 1e-12) throw new Error('vecteur nul');
  return [a[0] / n, a[1] / n, a[2] / n];
};
export const neg = (a) => [-a[0], -a[1], -a[2]];
export const deg = (r) => (r * 180) / Math.PI;
export const rad = (d) => (d * Math.PI) / 180;
export const comb = (a, s, b, t) => [a[0] * s + b[0] * t, a[1] * s + b[1] * t, a[2] * s + b[2] * t];

export const I4 = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

export function mat4Mul(A, B) {
  const C = new Array(16).fill(0);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j];
    C[i * 4 + j] = s;
  }
  return C;
}
export function mat4InvRigid(A) {
  const R = [A[0], A[4], A[8], A[1], A[5], A[9], A[2], A[6], A[10]];
  const t = [A[3], A[7], A[11]];
  const ti = [-(R[0] * t[0] + R[1] * t[1] + R[2] * t[2]), -(R[3] * t[0] + R[4] * t[1] + R[5] * t[2]), -(R[6] * t[0] + R[7] * t[1] + R[8] * t[2])];
  return [R[0], R[1], R[2], ti[0], R[3], R[4], R[5], ti[1], R[6], R[7], R[8], ti[2], 0, 0, 0, 1];
}
export const mat4Point = (A, p) => [A[0] * p[0] + A[1] * p[1] + A[2] * p[2] + A[3], A[4] * p[0] + A[5] * p[1] + A[6] * p[2] + A[7], A[8] * p[0] + A[9] * p[1] + A[10] * p[2] + A[11]];
export const mat4Vec = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[4] * v[0] + A[5] * v[1] + A[6] * v[2], A[8] * v[0] + A[9] * v[1] + A[10] * v[2]];
export const mat4Trans = (A) => [A[3], A[7], A[11]];
export const mat4Col = (A, j) => [A[j], A[4 + j], A[8 + j]];

// matrice 4x4 depuis une rotation 3x3 (ligne par ligne, 9 éléments) et une translation
export function mat4From(R, p) {
  R = R || [1, 0, 0, 0, 1, 0, 0, 0, 1];
  p = p || [0, 0, 0];
  return [R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1];
}
export function mat3Mul(A, B) {
  const C = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0;
    for (let k = 0; k < 3; k++) s += A[i * 3 + k] * B[k * 3 + j];
    C[i * 3 + j] = s;
  }
  return C;
}
export const Rx = (a) => { const c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; };
export const Ry = (a) => { const c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
export const Rz = (a) => { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; };
// rotation d'angle a (rad) autour d'un axe unitaire (Rodrigues)
export function rotAxe(axe, a) {
  const u = unit(axe);
  const K = [0, -u[2], u[1], u[2], 0, -u[0], -u[1], u[0], 0];
  const K2 = mat3Mul(K, K);
  const s = Math.sin(a), c = 1 - Math.cos(a);
  const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  return I.map((v, i) => v + s * K[i] + c * K2[i]);
}

// ---- géométrie de droites
export const projDroite = (P, p0, u) => add(p0, mul(u, dot(sub(P, p0), u)));
export const distDroite = (P, p0, u) => norm(sub(P, projDroite(P, p0, u)));
export function angleSigne(a, b, axe) {
  return Math.atan2(dot(cross(a, b), axe), dot(a, b));
}
// pieds de la normale commune de deux droites non parallèles
export function piedsNormaleCommune(p1, u1, p2, u2) {
  const n = cross(u1, u2);
  const n2 = dot(n, n);
  const w = sub(p2, p1);
  const t1 = dot(cross(w, u2), n) / n2;
  const t2 = dot(cross(w, u1), n) / n2;
  return [add(p1, mul(u1, t1)), add(p2, mul(u2, t2))];
}
