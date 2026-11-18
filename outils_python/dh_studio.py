#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dh_studio.py — Atelier Denavit-Hartenberg (MIRA · UE3)

Outil autonome (Python + numpy, aucune autre dépendance obligatoire) qui, pour un robot série décrit en URDF :

  * ouvre le modèle 3D (maillages STL/OBJ/DAE ou primitives de l'URDF) dans le navigateur ;
  * laisse l'étudiant PLACER les repères de la convention DH standard (clic sur le modèle ou saisie), vérifie les
    règles DH1/DH2 et CALCULE theta0, d, r, alpha à partir de ses repères ;
  * propose des aides progressives, et (mode enseignant) la solution de référence ;
  * exporte le tableau (CSV, JSON, LaTeX) et une image du robot avec repères.

Convention : A_i = Rot(z,theta_i) Trans(z,d_i) Trans(x,r_i) Rot(x,alpha_i),  theta_i = theta0_i + q_i.
Unités : mm et degrés.  Chaîne série : articulations rotoïdes (revolute / continuous) et prismatiques (prismatic).

Utilisation
-----------
  python dh_studio.py serve robot.urdf            # application locale (ouvre le navigateur)
  python dh_studio.py serve robot.urdf --solution # mode enseignant : bouton « solution » actif
  python dh_studio.py table robot.urdf            # tableau DH de référence (--format text|csv|json|tex)
  python dh_studio.py check robot.urdf reperes.json   # vérifie des repères saisis dans un fichier
  python dh_studio.py valider robot.urdf tableau.csv  # valide un tableau DH contre la cinématique de l'URDF
"""
import argparse
import json
import math
import os
import re
import struct
import sys
import threading
import webbrowser
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np

# ============================================================================
#  1. Petits outils d'algèbre
# ============================================================================


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def _T(R=None, p=None):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    if p is not None:
        M[:3, 3] = p
    return M


def rot_axe(axe, a):
    """Rotation d'angle a (rad) autour d'un axe unitaire (Rodrigues)."""
    axe = _u(axe)
    K = np.array([[0, -axe[2], axe[1]], [axe[2], 0, -axe[0]], [-axe[1], axe[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def _u(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("vecteur nul")
    return v / n


def _angle_signe(a, b, axe):
    return math.atan2(float(np.dot(np.cross(a, b), axe)), float(np.dot(a, b)))


def _proj_droite(P, p0, u):
    return p0 + np.dot(P - p0, u) * u


def _dist_droite(P, p0, u):
    return float(np.linalg.norm(P - _proj_droite(P, p0, u)))


def _pieds_normale_commune(p1, u1, p2, u2):
    n = np.cross(u1, u2)
    n2 = float(np.dot(n, n))
    w = p2 - p1
    t1 = np.dot(np.cross(w, u2), n) / n2
    t2 = np.dot(np.cross(w, u1), n) / n2
    return p1 + t1 * u1, p2 + t2 * u2


# tolérances géométriques : les URDF arrondissent souvent les angles (1.5708 au lieu de pi/2) → 1e-5 rad de bruit
TOL_PARALLELE = 2e-3   # |u1 x u2| en dessous duquel deux axes sont considérés parallèles (~0,11°)
TOL_SECANT = 0.1       # mm : distance entre axes en dessous de laquelle ils sont considérés sécants


def _nature_axes(p1, u1, p2, u2):
    if np.linalg.norm(np.cross(u1, u2)) < TOL_PARALLELE:
        return "parallèles"
    f1, f2 = _pieds_normale_commune(p1, u1, p2, u2)
    return "sécants" if np.linalg.norm(f2 - f1) < TOL_SECANT else "non coplanaires (gauches)"


def dh_matrice(theta_deg, d, r, alpha_deg):
    t, a = math.radians(theta_deg), math.radians(alpha_deg)
    ct, st, ca, sa = math.cos(t), math.sin(t), math.cos(a), math.sin(a)
    return np.array([[ct, -st * ca, st * sa, r * ct],
                     [st, ct * ca, -ct * sa, r * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1.0]])


# ============================================================================
#  2. Maillages (STL, OBJ, DAE si trimesh, primitives)
# ============================================================================


def _lire_stl(chemin):
    data = open(chemin, "rb").read()
    n = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n and n > 0:
        dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        arr = np.frombuffer(data, dtype=dt, count=n, offset=84)
        V = arr["v"].reshape(-1, 3).astype(float)
    else:                                                       # STL ASCII
        txt = data.decode("latin-1")
        V = np.array(re.findall(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", txt), dtype=float)
    F = np.arange(len(V)).reshape(-1, 3)
    return V, F


def _lire_obj(chemin):
    V, F = [], []
    for ligne in open(chemin, errors="ignore"):
        p = ligne.split()
        if not p:
            continue
        if p[0] == "v":
            V.append([float(x) for x in p[1:4]])
        elif p[0] == "f":
            idx = [int(t.split("/")[0]) for t in p[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1):
                F.append([idx[0], idx[k], idx[k + 1]])
    return np.array(V, float), np.array(F, int)


def _lire_via_trimesh(chemin):
    import trimesh
    m = trimesh.load(chemin, force="mesh")
    return np.asarray(m.vertices, float), np.asarray(m.faces, int)


def _fusionner(V, F, tol=1e-7):
    Vq = np.round(V / max(tol, 1e-9)).astype(np.int64)
    _, idx, inv = np.unique(Vq, axis=0, return_index=True, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    F2 = inv[F]
    ok = (F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])
    return V[idx], F2[ok]


def _decimer(V, F, max_faces):
    """Réduit le nombre de faces (fast_simplification si disponible, sinon regroupement de sommets)."""
    if len(F) <= max_faces:
        return V, F
    try:
        import fast_simplification
        Vs, Fs = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32),
                                              target_reduction=1 - max_faces / len(F))
        return Vs.astype(float), Fs.astype(int)
    except Exception:
        pass
    mn, mx = V.min(0), V.max(0)
    diag = float(np.linalg.norm(mx - mn)) or 1.0
    k = 40
    while True:
        cell = diag / k
        key = np.floor((V - mn) / cell).astype(np.int64)
        _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        inv = np.asarray(inv).reshape(-1)
        Vn = np.zeros((len(cnt), 3))
        np.add.at(Vn, inv, V)
        Vn /= cnt[:, None]
        Fn = inv[F]
        ok = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
        Fn = Fn[ok]
        if len(Fn) <= max_faces or k <= 4:
            return Vn, Fn
        k = int(k * 0.8)


def _primitive(geom):
    """Maillage d'une primitive URDF (box, cylinder, sphere) en mètres."""
    if geom.tag == "box":
        sx, sy, sz = [float(x) / 2 for x in geom.get("size").split()]
        V = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
        F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
        return V, F
    if geom.tag == "cylinder":
        r, h = float(geom.get("radius")), float(geom.get("length"))
        m = 24
        a = np.linspace(0, 2 * math.pi, m, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
        V = np.vstack([np.c_[ring, -h / 2 * np.ones(m)], np.c_[ring, h / 2 * np.ones(m)], [[0, 0, -h / 2], [0, 0, h / 2]]])
        F = []
        for i in range(m):
            j = (i + 1) % m
            F += [[i, j, m + j], [i, m + j, m + i], [2 * m, j, i], [2 * m + 1, m + i, m + j]]
        return V, np.array(F)
    if geom.tag == "sphere":
        r = float(geom.get("radius"))
        nu, nv = 14, 10
        V = [[0, 0, r], [0, 0, -r]]
        for iv in range(1, nv):
            ph = math.pi * iv / nv
            for iu in range(nu):
                th = 2 * math.pi * iu / nu
                V.append([r * math.sin(ph) * math.cos(th), r * math.sin(ph) * math.sin(th), r * math.cos(ph)])
        V = np.array(V)
        F = []
        idx = lambda iv, iu: 2 + (iv - 1) * nu + (iu % nu)
        for iu in range(nu):
            F.append([0, idx(1, iu), idx(1, iu + 1)])
            F.append([1, idx(nv - 1, iu + 1), idx(nv - 1, iu)])
        for iv in range(1, nv - 1):
            for iu in range(nu):
                a, b, c, d = idx(iv, iu), idx(iv, iu + 1), idx(iv + 1, iu + 1), idx(iv + 1, iu)
                F += [[a, b, c], [a, c, d]]
        return V, np.array(F)
    return None


def _resoudre_chemin(nom, dossier_urdf):
    """Résout filename="package://pkg/…" ou chemin relatif en cherchant autour du fichier URDF."""
    nom = nom.strip()
    reste = re.sub(r"^(package://[^/]+/|file://)", "", nom)
    cands = [os.path.join(dossier_urdf, reste), reste]
    base = dossier_urdf
    for _ in range(4):
        base = os.path.dirname(base)
        cands.append(os.path.join(base, reste))
    for c in cands:
        if os.path.isfile(c):
            return c
    bn = os.path.basename(reste)
    base = dossier_urdf
    for _ in range(3):
        base = os.path.dirname(base)
        for racine, _, fichiers in os.walk(base):
            if bn in fichiers:
                return os.path.join(racine, bn)
    return None


# ============================================================================
#  3. Le robot (URDF -> chaîne série)
# ============================================================================


def _vec(s, defaut):
    return np.array([float(x) for x in s.split()], float) if s else np.array(defaut, float)


def _origine(el):
    o = el.find("origin") if el is not None else None
    xyz = _vec(o.get("xyz") if o is not None else None, [0, 0, 0])
    rpy = _vec(o.get("rpy") if o is not None else None, [0, 0, 0])
    return _T(Rz(rpy[2]) @ Ry(rpy[1]) @ Rx(rpy[0]), xyz)


class Robot:
    """Chaîne série extraite d'un URDF. Unités internes : mm / degrés."""

    def __init__(self, urdf, tip=None, base=None, mesh_faces=5000, avec_meshes=True, verbose=True):
        self.chemin = os.path.abspath(urdf)
        self.dossier = os.path.dirname(self.chemin)
        brut = open(self.chemin, encoding="utf-8", errors="ignore").read()
        if "xmlns:xacro" in brut or "<xacro:" in brut or "${" in brut:
            raise ValueError("Fichier xacro : convertissez-le d'abord en URDF (xacro robot.urdf.xacro > robot.urdf).")
        root = ET.fromstring(brut.encode("utf-8")) if not brut.lstrip().startswith("<?xml") else ET.parse(self.chemin).getroot()
        self.nom = root.get("name", os.path.basename(urdf))
        liens = {l.get("name"): l for l in root.findall("link")}
        joints = []
        for j in root.findall("joint"):
            lim = j.find("limit")
            ax = j.find("axis")
            joints.append(dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=_origine(j), axe=_vec(ax.get("xyz") if ax is not None else None, [1, 0, 0]),
                lo=float(lim.get("lower")) if (lim is not None and lim.get("lower") is not None) else None,
                hi=float(lim.get("upper")) if (lim is not None and lim.get("upper") is not None) else None,
                mimic=j.find("mimic") is not None))
        enfants = {}
        for j in joints:
            enfants.setdefault(j["parent"], []).append(j)
        fils = {j["child"] for j in joints}
        racines = [n for n in liens if n not in fils]
        if base is None:
            if not racines:
                raise ValueError("Aucun lien racine dans l'URDF.")
            base = racines[0]
        # chemins racine -> feuilles
        chemins = []

        def dfs(lien, acc):
            suite = enfants.get(lien, [])
            if not suite:
                chemins.append(list(acc))
            for j in suite:
                acc.append(j)
                dfs(j["child"], acc)
                acc.pop()
        dfs(base, [])
        ROT = ("revolute", "continuous")

        def utile(ch):
            """Chaîne tronquée après la dernière rotoïde (+ fixes qui suivent). Une prismatique terminale est conservée
            si elle est seule à partir de son lien parent et ne copie pas une autre articulation (mimic)."""
            der = -1
            for k, j in enumerate(ch):
                if j["type"] in ROT:
                    der = k
            k = der + 1
            while k < len(ch) and (ch[k]["type"] == "fixed" or (ch[k]["type"] == "prismatic" and not ch[k]["mimic"]
                                                                and len(enfants.get(ch[k]["parent"], [])) == 1)):
                if ch[k]["type"] == "prismatic":
                    der = k
                k += 1
            if der < 0:
                return list(ch)
            fin = der
            while fin + 1 < len(ch) and ch[fin + 1]["type"] == "fixed":
                fin += 1
            return ch[:fin + 1]

        utiles, vus = [], set()
        for c in chemins:
            u = utile(c)
            cle = tuple(j["name"] for j in u)
            if cle not in vus:
                vus.add(cle)
                utiles.append(u)
        nrot = lambda ch: sum(1 for j in ch if j["type"] in ROT)
        npris = lambda ch: sum(1 for j in ch if j["type"] == "prismatic")
        if tip is not None:
            cand = [c for c in utiles if c and c[-1]["child"] == tip] or [c for c in chemins if c and c[-1]["child"] == tip]
            if not cand:
                raise ValueError(f"Lien terminal « {tip} » introuvable.")
            chaine = cand[0]
        else:
            # le plus d'articulations rotoïdes, puis le moins de prismatiques (le bras plutôt que les doigts de pince)
            chaine = utiles[0]
            for c in utiles[1:]:
                if (nrot(c), -npris(c)) > (nrot(chaine), -npris(chaine)):
                    chaine = c
            if len(chemins) > 1 and verbose:
                print(f"Attention : l'URDF est ramifié ({len(chemins)} branches) ; branche retenue : jusqu'à « {chaine[-1]['child']} ». "
                      f"Utilisez --tip pour en choisir une autre.", file=sys.stderr)
        for j in chaine:
            if j["type"] not in ("revolute", "continuous", "prismatic", "fixed"):
                raise ValueError(f"Type d'articulation non géré : {j['type']} ({j['name']}).")
        self.base, self.tip = base, chaine[-1]["child"] if chaine else base
        # repères de la chaîne
        self.ORIG, self.AXL, self.noms, self.limites, self.types = [], [], [], [], []
        attach = [(base, 0, np.eye(4))]
        Tacc = np.eye(4)
        m = 0
        for j in chaine:
            To = j["T"].copy()
            To[:3, 3] *= 1000.0
            if j["type"] == "fixed":
                Tacc = Tacc @ To
                attach.append((j["child"], m, Tacc.copy()))
            else:
                self.ORIG.append(Tacc @ To)
                self.AXL.append(_u(j["axe"]))
                self.noms.append(j["name"])
                self.types.append("P" if j["type"] == "prismatic" else "R")
                if j["type"] == "prismatic":   # millimètres
                    self.limites.append((-100.0, 100.0) if j["lo"] is None or j["hi"] is None else (j["lo"] * 1000.0, j["hi"] * 1000.0))
                elif j["type"] == "continuous" or j["lo"] is None or j["hi"] is None:   # degrés
                    self.limites.append((-180.0, 180.0))
                else:
                    self.limites.append((math.degrees(j["lo"]), math.degrees(j["hi"])))
                m += 1
                Tacc = np.eye(4)
                attach.append((j["child"], m, np.eye(4)))
        self.n = m
        if self.n < 1:
            raise ValueError("Aucune articulation mobile sur la chaîne.")
        self.TIP = Tacc
        self.F0 = self.fk(np.zeros(self.n))
        # axes z_k (k = 0..n-1) = axe de l'articulation k+1, à q = 0 ; z_n = z_{n-1}
        self.AXES = []
        for k in range(self.n):
            To = self.F0[k] @ self.ORIG[k]
            self.AXES.append((To[:3, 3].copy(), _u(To[:3, :3] @ self.AXL[k])))
        self.AXES.append(self.AXES[-1])
        Ttip = self.F0[self.n] @ self.TIP
        pn, un = self.AXES[-1]
        self.OUTIL = _proj_droite(Ttip[:3, 3], pn, un)
        # repère 0 imposé
        p0, z0 = self.AXES[0]
        O0 = _proj_droite(np.zeros(3), p0, z0)
        x0 = None
        for cand in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
            v = cand - np.dot(cand, z0) * z0
            if np.linalg.norm(v) > 0.5:
                x0 = _u(v)
                break
        self.REP0 = dict(O=O0, x=x0, z=z0.copy())
        self.meshes = self._charger_meshes(liens, attach, mesh_faces, verbose) if avec_meshes else [(np.zeros((0, 3)), np.zeros((0, 3), int)) for _ in range(self.n + 1)]
        self.taille = self._taille()

    # ----------------------------------------------------------- cinématique
    def fk(self, q):
        """q : degrés pour une rotoïde, millimètres pour une prismatique."""
        q = np.asarray(q, float)
        F = [np.eye(4)]
        for k in range(self.n):
            if self.types[k] == "P":
                mouv = np.eye(4)
                mouv[:3, 3] = self.AXL[k] * q[k]
            else:
                mouv = _T(rot_axe(self.AXL[k], math.radians(q[k])))
            F.append(F[-1] @ self.ORIG[k] @ mouv)
        return F

    def _taille(self):
        pts = [self.F0[k][:3, 3] for k in range(self.n + 1)] + [self.OUTIL]
        for m, (V, _) in enumerate(self.meshes):
            if len(V):
                pts.append((self.F0[m][:3, :3] @ V.mean(0)) + self.F0[m][:3, 3])
        pts = np.array(pts)
        return float(max(np.linalg.norm(pts.max(0) - pts.min(0)), 300.0))

    # ---------------------------------------------------------------- maillages
    def _charger_meshes(self, liens, attach, mesh_faces, verbose):
        groupes = [([], []) for _ in range(self.n + 1)]
        manquants = set()
        for nom_lien, m, Tl in attach:
            el = liens.get(nom_lien)
            if el is None:
                continue
            for vis in el.findall("visual"):
                geom = vis.find("geometry")
                if geom is None or len(geom) == 0:
                    continue
                g = geom[0]
                Tv = _origine(vis)
                if g.tag == "mesh":
                    chemin = _resoudre_chemin(g.get("filename", ""), self.dossier)
                    if chemin is None:
                        manquants.add(g.get("filename"))
                        continue
                    ext = os.path.splitext(chemin)[1].lower()
                    try:
                        if ext == ".stl":
                            V, F = _lire_stl(chemin)
                        elif ext == ".obj":
                            V, F = _lire_obj(chemin)
                        else:
                            V, F = _lire_via_trimesh(chemin)
                    except Exception as e:
                        manquants.add(f"{g.get('filename')} ({type(e).__name__})")
                        continue
                    sc = _vec(g.get("scale"), [1, 1, 1])
                    V = V * sc
                    V, F = _fusionner(V, F, tol=1e-7)
                else:
                    pr = _primitive(g)
                    if pr is None:
                        continue
                    V, F = pr
                V, F = _decimer(V, F, mesh_faces)
                Vm = V * 1000.0
                Tv_mm = Tv.copy()
                Tv_mm[:3, 3] *= 1000.0           # la translation de l'URDF est en mètres ; Tl est déjà en mm
                Tt = Tl @ Tv_mm
                Vw = Vm @ Tt[:3, :3].T + Tt[:3, 3]
                off = sum(len(v) for v in groupes[m][0])
                groupes[m][0].append(Vw)
                groupes[m][1].append(F + off)
        if manquants and verbose:
            print("Maillages introuvables / illisibles (ignorés) :", ", ".join(sorted(manquants)), file=sys.stderr)
        out = []
        for Vs, Fs in groupes:
            if Vs:
                out.append((np.vstack(Vs), np.vstack(Fs)))
            else:
                out.append((np.zeros((0, 3)), np.zeros((0, 3), int)))
        return out

    # --------------------------------------------------------- solution de référence
    def solution_reperes(self):
        """Repères DH canoniques (liste de n+1 dict {O, x, z}) construits par normales communes."""
        reps = [dict(self.REP0)]
        for i in range(1, self.n + 1):
            zp_pt, zp = self.AXES[i - 1]
            zc_pt, zc = self.AXES[i]
            Op, xp = reps[-1]["O"], reps[-1]["x"]
            if i == self.n:
                Oi, xi = self.OUTIL.copy(), xp.copy()
            else:
                nrm = np.cross(zp, zc)
                if np.linalg.norm(nrm) > TOL_PARALLELE:
                    f1, f2 = _pieds_normale_commune(zp_pt, zp, zc_pt, zc)
                    Oi = f2
                    rho = f2 - f1
                    if np.linalg.norm(rho) > TOL_SECANT:
                        xi = _u(rho)
                    else:
                        xi = _u(nrm)
                        if np.dot(xi, xp) < 0:
                            xi = -xi
                else:
                    Oi = _proj_droite(Op, zc_pt, zc)
                    rho = Oi - Op
                    xi = _u(rho) if np.linalg.norm(rho) > TOL_SECANT else xp.copy()
            reps.append(dict(O=Oi, x=xi, z=zc.copy()))
        return reps

    @staticmethod
    def parametres(rp, r):
        zp, xp, zc, xc = rp["z"], rp["x"], r["z"], r["x"]
        w = r["O"] - rp["O"]
        return np.array([math.degrees(_angle_signe(xp, xc, zp)), float(np.dot(w, zp)), float(np.dot(w, xc)),
                         math.degrees(_angle_signe(zp, zc, xc))])

    def tableau_reference(self):
        R = self.solution_reperes()
        return np.array([self.parametres(R[i - 1], R[i]) for i in range(1, self.n + 1)])

    # ------------------------------------------------- analyse des repères de l'étudiant
    def analyser(self, reperes, tol_pos=10.0, tol_ang=6.0):
        """reperes : {i: {"O": [x,y,z] | None, "x": [ux,uy,uz] | None}} pour i = 1..n (mm, base, q = 0).
        Retourne {ok, tableau, reperes, etats:[{i, statut ('ok'|'err'|'warn'|'todo'), msgs:[...], params}]}"""
        reps = [dict(self.REP0)]
        etats, lignes = [], []
        bloque = False
        for i in range(1, self.n + 1):
            zp_pt, zp = self.AXES[i - 1]
            zc_pt, zc = self.AXES[i]
            ent = dict(i=i, statut="todo", msgs=[], params=None)
            etats.append(ent)
            if bloque:
                ent["msgs"].append("En attente des repères précédents.")
                continue
            rep = (reperes or {}).get(i) or (reperes or {}).get(str(i)) or {}
            try:
                O_raw = np.asarray(rep.get("O"), float)
                x_raw = np.asarray(rep.get("x"), float)
                if O_raw.shape != (3,) or x_raw.shape != (3,) or np.isnan(O_raw).any() or np.isnan(x_raw).any():
                    raise ValueError
                if np.linalg.norm(x_raw) < 1e-9:
                    ent["statut"] = "err"; ent["msgs"].append("Le vecteur x est nul."); bloque = True
                    continue
            except Exception:
                ent["msgs"].append("À compléter : origine O et direction x.")
                bloque = True
                continue
            ok_i = True
            x_raw = _u(x_raw)
            # O_i sur l'axe z_i
            dax = _dist_droite(O_raw, zc_pt, zc)
            if dax > tol_pos:
                ent["msgs"].append(f"L'origine O_{i} est à {dax:.0f} mm de l'axe z_{i} (axe de l'articulation {min(i + 1, self.n)}) : elle doit être SUR cet axe.")
                ok_i = False
            O = _proj_droite(O_raw, zc_pt, zc)
            Op, xp = reps[-1]["O"], reps[-1]["x"]
            # « accrochage » sur la position exacte quand elle est unique
            if i == self.n:
                if np.linalg.norm(O - self.OUTIL) <= tol_pos:
                    O = self.OUTIL.copy()
            elif np.linalg.norm(np.cross(zp, zc)) > TOL_PARALLELE:
                f1, f2 = _pieds_normale_commune(zp_pt, zp, zc_pt, zc)
                if np.linalg.norm(O - f2) <= tol_pos:
                    O = f2
                else:
                    ent["msgs"].append(f"O_{i} doit être au pied de la normale commune à z_{i-1} et z_{i} (vous êtes à {np.linalg.norm(O - f2):.0f} mm de ce point).")
                    ok_i = False
            else:
                Oz = _proj_droite(Op, zc_pt, zc)
                if np.linalg.norm(O - Oz) <= tol_pos:
                    O = Oz
            # DH1
            ang_perp = math.degrees(math.asin(min(1.0, abs(float(np.dot(x_raw, zp))))))
            if ang_perp > tol_ang:
                ent["msgs"].append(f"(DH1) x_{i} n'est pas perpendiculaire à z_{i-1} (écart {ang_perp:.0f}°).")
                ok_i = False
            x = x_raw - np.dot(x_raw, zp) * zp
            if np.linalg.norm(x) < 1e-9:
                ent["statut"] = "err"; ent["msgs"].append(f"x_{i} est colinéaire à z_{i-1}."); bloque = True
                continue
            x = _u(x)
            w = O - Op
            d = float(np.dot(w, zp))
            rho = w - d * zp
            nrho = float(np.linalg.norm(rho))
            if i < self.n and nrho > 3.0:
                ang_rho = math.degrees(math.acos(min(1.0, abs(float(np.dot(_u(rho), x))))))
                if ang_rho > tol_ang:
                    ent["msgs"].append(f"(DH2) x_{i} ne rencontre pas z_{i-1} : le vecteur O_{i-1}→O_{i} n'est pas dans la direction de x_{i} (écart {ang_rho:.0f}°).")
                    ok_i = False
                else:
                    x = _u(rho) if np.dot(_u(rho), x) > 0 else -_u(rho)
                    if abs(float(np.dot(rho, zc))) > tol_pos:
                        ent["msgs"].append(f"x_{i} doit aussi être perpendiculaire à z_{i} : c'est la normale COMMUNE à z_{i-1} et z_{i}.")
                        ok_i = False
            elif i < self.n:
                nrm = np.cross(zp, zc)
                if np.linalg.norm(nrm) > TOL_PARALLELE:
                    ang_n = math.degrees(math.acos(min(1.0, abs(float(np.dot(x, _u(nrm)))))))
                    if ang_n > tol_ang:
                        ent["msgs"].append(f"z_{i-1} et z_{i} sont sécants (en O_{i}) : x_{i} doit être perpendiculaire aux deux axes (écart {ang_n:.0f}°).")
                        ok_i = False
                    else:
                        x = _u(nrm) if np.dot(_u(nrm), x) > 0 else -_u(nrm)
            if i == self.n and nrho <= 3.0 and math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(x, xp)))))) <= tol_ang:
                x = xp.copy()
            if i == self.n:
                dt = float(np.linalg.norm(O - self.OUTIL))
                if dt > tol_pos:
                    ent["msgs"].append(f"O_{i} est à {dt:.0f} mm de l'origine de l'outil (bride) ; le tableau reste cohérent mais l'outil n'est pas à la bride.")
                    ent["statut"] = "warn"
            r_ = dict(O=O, x=x, z=zc.copy())
            p = self.parametres(dict(O=Op, x=xp, z=zp), r_)
            reps.append(r_)
            lignes.append(p)
            ent["params"] = [float(v) for v in p]
            if ok_i:
                ent["statut"] = "warn" if ent["statut"] == "warn" else "ok"
                if r_ and float(np.dot(O - Op, x)) < -1e-6 and ent["statut"] == "ok":
                    ent["msgs"].append("r négatif : valide (x_i pointe à l'opposé de la normale commune) ; vous pouvez inverser x_i pour avoir r > 0.")
            else:
                ent["statut"] = "err"
        complet = len(lignes) == self.n
        ok = complet and all(e["statut"] in ("ok", "warn") for e in etats)
        return dict(ok=ok, tableau=np.array(lignes).tolist(), reperes=[dict(O=r["O"].tolist(), x=r["x"].tolist(), z=r["z"].tolist()) for r in reps],
                    etats=etats)

    # ------------------------------------------- construction guidée (curseurs)
    def construire(self, params, tol_pos=10.0, tol_ang=6.0):
        """Construit les repères à partir de choix simples et renvoie leur analyse.

        params : {i: {"t": position de O_i sur l'axe z_i (mm, depuis le point de référence de l'axe) ou None,
                      "sign": +1/-1 (sens de x_i quand sa direction est imposée),
                      "phi": angle de x_i par rapport à x_{i-1}, autour de z_{i-1} (deg), quand x_i est libre}}
        Règles : O_i est sur z_i (un seul degré de liberté, t) ; si z_{i-1} et z_i sont sécants, O_i est imposé à
        leur intersection ; x_i est perpendiculaire à z_{i-1} (DH1) et doit rencontrer z_{i-1} (DH2) : une fois O_i
        placé, sa direction est imposée (au signe près) sauf si O_i est sur z_{i-1} avec z_i parallèle (angle libre).
        """
        params = params or {}
        L = max(500.0, 0.9 * self.taille)
        reperes, frames = {}, []
        prev = dict(self.REP0)
        bloque = False
        for i in range(1, self.n + 1):
            zp_pt, zp = self.AXES[i - 1]
            zc_pt, zc = self.AXES[i]
            pr = params.get(i) or params.get(str(i)) or {}
            nrm = np.cross(zp, zc)
            parallele = np.linalg.norm(nrm) < TOL_PARALLELE
            fr = dict(i=i, bloque=bloque, origine=dict(mode="libre", t=None, tmin=-L, tmax=L, nature=None), x=dict(mode=None),
                      O=None, xv=None, axe_pt=zc_pt.tolist(), axe_u=zc.tolist())
            frames.append(fr)
            if bloque:
                continue
            Op, xp = prev["O"], prev["x"]
            # ---- origine
            t = None
            if i < self.n and not parallele:
                f1, f2 = _pieds_normale_commune(zp_pt, zp, zc_pt, zc)
                if np.linalg.norm(f2 - f1) < TOL_SECANT:
                    t = float(np.dot(f2 - zc_pt, zc))
                    fr["origine"].update(mode="force", t=t, nature="sécants")
            if t is None:
                tv = pr.get("t")
                if tv is None:
                    bloque = True
                    fr["bloque"] = False        # ce repère est « à placer » ; les suivants attendront
                    fr["attente_origine"] = True
                    continue
                t = float(tv)
                fr["origine"]["t"] = t
            O = zc_pt + t * zc
            # ---- direction de x
            w = O - Op
            d = float(np.dot(w, zp))
            rho = w - d * zp
            nr = float(np.linalg.norm(rho))
            if nr > 3.0:
                base, mode, nat = rho / nr, "force", "normale"
            elif not parallele:
                base = _u(nrm)
                if np.dot(base, xp) < 0:
                    base = -base
                mode, nat = "force", "sécants"
            else:
                base, mode, nat = None, "libre", None
            if mode == "force":
                sg = -1 if (pr.get("sign") in (-1, "-1")) else 1
                x = sg * base
                fr["x"] = dict(mode="force", nature=nat, sign=sg)
            else:
                e1 = xp - np.dot(xp, zp) * zp
                e1 = _u(e1) if np.linalg.norm(e1) > 1e-9 else _u(np.cross(zp, [0.3, 0.5, 0.8]))
                e2 = np.cross(zp, e1)
                phi = float(pr.get("phi", 0.0) or 0.0)
                x = math.cos(math.radians(phi)) * e1 + math.sin(math.radians(phi)) * e2
                fr["x"] = dict(mode="libre", phi=phi)
            fr["O"], fr["xv"] = O.tolist(), x.tolist()
            reperes[i] = dict(O=O.tolist(), x=x.tolist())
            res = self.analyser(reperes, tol_pos, tol_ang)
            if len(res["reperes"]) > i:
                prev = dict(res["reperes"][i])
                prev["O"], prev["x"], prev["z"] = np.array(prev["O"]), np.array(prev["x"]), np.array(prev["z"])
            else:
                bloque = True
        return dict(frames=frames, analyse=self.analyser(reperes, tol_pos, tol_ang))

    def parametres_solution(self):
        """Valeurs de curseurs (t, sign, phi) qui reproduisent la solution de référence."""
        R = self.solution_reperes()
        params = {}
        for i in range(1, self.n + 1):
            zp_pt, zp = self.AXES[i - 1]
            zc_pt, zc = self.AXES[i]
            params[i] = dict(t=float(np.dot(R[i]["O"] - zc_pt, zc)), sign=1,
                             phi=math.degrees(_angle_signe(R[i - 1]["x"], R[i]["x"], zp)))
            xc = self.construire(params)["frames"][i - 1]["xv"]
            if xc is not None and np.dot(xc, R[i]["x"]) < 0:
                params[i]["sign"] = -1
        return params

    # ------------------------------------------------------------------ aides
    def aide(self, i, niveau, max_niveau=3):
        niveau = min(niveau, max_niveau)
        if not 1 <= i <= self.n:
            return f"Choisissez un repère entre 1 et {self.n}."
        R = self.solution_reperes()
        zp_pt, zp = self.AXES[i - 1]
        zc_pt, zc = self.AXES[i]
        out = []
        if i == self.n:
            out.append(f"Repère {i} (dernier) : z_{i} est confondu avec z_{i-1} (direction d'approche de l'outil). L'origine O_{i} se place au centre de la bride, "
                       f"sur l'axe de la dernière articulation ; x_{i} est libre (perpendiculaire à z_{i}).")
            if niveau >= 3:
                out.append(f"Réponse : O_{i} = {np.round(R[i]['O'], 1).tolist()} mm ; x_{i} = {np.round(R[i]['x'], 3).tolist()}.")
            return "\n".join(out)
        nat = _nature_axes(zp_pt, zp, zc_pt, zc)
        out.append(f"Repère {i} : on regarde les axes z_{i-1} (articulation {i}) et z_{i} (articulation {i+1}).")
        if niveau >= 1:
            out.append(f"Niveau 1 — ces deux axes sont {nat}.")
        if niveau >= 2:
            if nat.startswith("parallèles"):
                out.append(f"Niveau 2 — axes parallèles : une infinité de normales communes ; on prend celle qui passe par O_{i-1} (d_{i} = 0). O_{i} est le projeté de O_{i-1} sur z_{i} ; x_{i} pointe de O_{i-1} vers O_{i}.")
            elif nat.startswith("sécants"):
                out.append(f"Niveau 2 — axes sécants : O_{i} est le point d'intersection ; x_{i} est perpendiculaire aux deux axes (produit vectoriel z_{i-1} × z_{i}), orienté comme x_{i-1} si possible.")
            else:
                out.append(f"Niveau 2 — axes gauches : x_{i} est porté par la normale commune, orientée de z_{i-1} vers z_{i} (r_{i} > 0) ; O_{i} est le pied de cette normale sur z_{i}.")
        if niveau >= 3:
            p = self.parametres(R[i - 1], R[i])
            out.append(f"Niveau 3 — O_{i} = {np.round(R[i]['O'], 1).tolist()} mm ; x_{i} = {np.round(R[i]['x'], 3).tolist()} → θ0={p[0]:+.2f}° d={p[1]:.2f} r={p[2]:.2f} α={p[3]:+.2f}°")
            if self.types[i - 1] == "P":
                out.append(f"Articulation {i} prismatique : la variable est d_{i} = d0_{i} + q_{i} (θ_{i} reste constant) ; d0 est la valeur lorsque q_{i} = 0.")
        return "\n".join(out)

    # ---------------------------------------------------- validation d'un tableau DH
    def valider_tableau(self, tableau, n_tests=15, tol_mm=0.5, tol_deg=0.1, graine=0):
        """Test : C_i = (0Ti_DH)^-1 · F_i(URDF) doit rester constant quand q varie. Retourne (ok, lignes de rapport)."""
        T = np.asarray(tableau, float)
        if T.shape != (self.n, 4):
            return False, [f"Le tableau doit avoir {self.n} lignes de 4 valeurs (θ0, d, r, α)."]
        rng = np.random.default_rng(graine)
        lo = np.array([a if self.types[k] == "P" else max(a, -180) for k, (a, b) in enumerate(self.limites)])
        hi = np.array([b if self.types[k] == "P" else min(b, 180) for k, (a, b) in enumerate(self.limites)])

        # repère 0 imposé (pas toujours confondu avec la base : z0 est l'axe de l'articulation 1, parfois orienté vers le bas)
        B = np.eye(4)
        B[:3, 0], B[:3, 2], B[:3, 3] = self.REP0["x"], self.REP0["z"], self.REP0["O"]
        B[:3, 1] = np.cross(self.REP0["z"], self.REP0["x"])

        def dh_fk(q):
            M, out = B.copy(), []
            for k in range(self.n):
                P = self.types[k] == "P"
                M = M @ dh_matrice(T[k, 0] + (0 if P else q[k]), T[k, 1] + (q[k] if P else 0), T[k, 2], T[k, 3])
                out.append(M.copy())
            return out
        T0, F0 = dh_fk(np.zeros(self.n)), self.F0
        C0 = [np.linalg.inv(T0[k]) @ F0[k + 1] for k in range(self.n)]
        pire = np.zeros((self.n, 2))
        for _ in range(n_tests):
            q = rng.uniform(lo, hi)
            Ts, F = dh_fk(q), self.fk(q)
            for k in range(self.n):
                C = np.linalg.inv(Ts[k]) @ F[k + 1]
                dp = np.linalg.norm(C[:3, 3] - C0[k][:3, 3])
                Rr = C[:3, :3].T @ C0[k][:3, :3]
                da = math.degrees(math.acos(max(-1, min(1, (np.trace(Rr) - 1) / 2))))
                pire[k] = np.maximum(pire[k], [dp, da])
        rapport, ok, premiere = [], True, None
        for k in range(self.n):
            bon = pire[k, 0] <= tol_mm and pire[k, 1] <= tol_deg
            ok &= bon
            if not bon and premiere is None:
                premiere = k + 1
            rapport.append(f"{'✔' if bon else '✘'} 0T{k + 1} : écart max {pire[k, 0]:.2f} mm, {pire[k, 1]:.2f}°")
        if ok:
            rapport.append("→ Tableau DH validé : il reproduit la cinématique de l'URDF.")
        elif premiere and premiere > 1:
            rapport.append(f"→ Première incohérence sur 0T{premiere} : erreur dans la ligne {premiere} ou la ligne {premiere - 1} (α_{premiere - 1} fixe l'axe z_{premiere - 1}).")
        else:
            rapport.append("→ Incohérence dès 0T1 : vérifiez la ligne 1 (signe des angles).")
        return ok, rapport

    # ------------------------------------------------------------------ exports
    def en_json(self, tableau, nom_robot=None):
        T = np.asarray(tableau, float)
        return json.dumps(dict(robot=nom_robot or self.nom, convention="DH standard", unites=dict(longueur="mm", angle="deg"),
                               lignes=[dict(i=i + 1, theta0=round(_nettoyer(T[i, 0]), 3) + 0.0, d=round(_nettoyer(T[i, 1]), 3) + 0.0, r=round(_nettoyer(T[i, 2]), 3) + 0.0, alpha=round(_nettoyer(T[i, 3]), 3) + 0.0,
                                            **({"variable": "d" if self.types[i] == "P" else "theta"} if "P" in self.types else {}))
                                       for i in range(len(T))]), ensure_ascii=False, indent=2)


def _nettoyer(v):
    """Écarte le bruit numérique (URDF aux angles arrondis) : |v| < 0,005 → 0."""
    return 0.0 if abs(v) < 0.005 else float(v)


def _fmt(v, nd=3):
    v = _nettoyer(v)
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def formater_tableau(tableau, fmt="text", nom_robot="robot", robot=None, types=None):
    T = np.asarray(tableau, float)
    types = types if types is not None else (robot.types if robot is not None else None)
    P = lambda k: bool(types) and types[k] == "P"
    av = bool(types) and "P" in types   # colonne « variable » seulement s'il y a une prismatique
    if fmt == "csv":
        return "i,theta0_deg,d_mm,r_mm,alpha_deg" + (",variable" if av else "") + "\n" + "\n".join(
            f"{i + 1},{_fmt(T[i, 0])},{_fmt(T[i, 1])},{_fmt(T[i, 2])},{_fmt(T[i, 3])}" + (("," + ("d" if P(i) else "theta")) if av else "") for i in range(len(T)))
    if fmt == "json":
        return (robot.en_json(T, nom_robot) if robot else json.dumps(T.tolist()))
    if fmt == "tex":
        lg = "\n".join(f"{i + 1} & {_fmt(T[i, 0], 2)} & {_fmt(T[i, 1], 2)} & {_fmt(T[i, 2], 2)} & {_fmt(T[i, 3], 2)}" + ((" & " + ("$d$" if P(i) else "$\\theta$")) if av else "") + " \\\\" for i in range(len(T)))
        return ("\\begin{tabular}{c|cccc" + ("c" if av else "") + "}\n$i$ & $\\theta_{0,i}$ (°) & $d_i$ (mm) & $r_i$ (mm) & $\\alpha_i$ (°)" + (" & variable" if av else "") + " \\\\ \\hline\n" + lg + "\n\\end{tabular}")
    out = [f"Tableau DH standard — {nom_robot}", " i |  theta0 (°) |    d (mm) |    r (mm) |  alpha (°)" + (" | variable" if av else ""),
           "---+-------------+-----------+-----------+-----------" + ("+---------" if av else "")]
    for i in range(len(T)):
        out.append(f"{i + 1:2d} | {T[i, 0]:11.3f} | {T[i, 1]:9.3f} | {T[i, 2]:9.3f} | {T[i, 3]:10.3f}" + ((" | " + ("d" if P(i) else "θ")) if av else ""))
    return "\n".join(out)


# ============================================================================
#  4. Serveur local
# ============================================================================
PALETTE = ["#9aa1a9", "#dfe3e8", "#c4c9d0", "#dfe3e8", "#c4c9d0", "#dfe3e8", "#c4c9d0", "#dfe3e8", "#c4c9d0"]


def donnees_robot(rb, aide_max, solution):
    liens = []
    for m, (V, F) in enumerate(rb.meshes):
        liens.append(dict(m=m, couleur=PALETTE[m % len(PALETTE)], verts=np.round(V, 2).ravel().tolist(), faces=F.astype(int).ravel().tolist()))
    return dict(nom=rb.nom, n=rb.n, taille=rb.taille, joints=[dict(nom=rb.noms[k], lo=rb.limites[k][0], hi=rb.limites[k][1], type=rb.types[k]) for k in range(rb.n)],
                axes=[dict(p=p.tolist(), u=u.tolist()) for p, u in rb.AXES], outil=rb.OUTIL.tolist(),
                rep0=dict(O=rb.REP0["O"].tolist(), x=rb.REP0["x"].tolist(), z=rb.REP0["z"].tolist()),
                F0=[F.ravel().tolist() for F in rb.F0], liens=liens, aide_max=aide_max, solution=solution)


def _reperes_depuis_json(d):
    out = {}
    for k, v in (d or {}).items():
        out[int(k)] = v
    return out


def faire_handler(rb, aide_max, solution, page):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _envoyer(self, code, contenu, typ="application/json; charset=utf-8", extra=None):
            if isinstance(contenu, str):
                contenu = contenu.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", typ)
            self.send_header("Content-Length", str(len(contenu)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(contenu)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/":
                self._envoyer(200, page.replace("__TITRE__", rb.nom), "text/html; charset=utf-8")
            elif u.path == "/plotly.min.js":
                try:
                    from plotly.offline import get_plotlyjs
                    self._envoyer(200, get_plotlyjs(), "application/javascript; charset=utf-8", {"Cache-Control": "max-age=86400"})
                except Exception:
                    self._envoyer(302, "", "text/plain", {"Location": "https://cdn.plot.ly/plotly-2.35.2.min.js"})
            elif u.path == "/api/robot":
                self._envoyer(200, json.dumps(donnees_robot(rb, aide_max, solution)))
            elif u.path == "/api/aide":
                i, niv = int(q.get("i", ["1"])[0]), int(q.get("niveau", ["1"])[0])
                self._envoyer(200, json.dumps(dict(texte=rb.aide(i, niv, aide_max))))
            elif u.path == "/api/solution":
                if not solution:
                    self._envoyer(403, json.dumps(dict(erreur="Mode enseignant désactivé (relancer avec --solution).")))
                else:
                    self._envoyer(200, json.dumps(dict(params={str(k): v for k, v in rb.parametres_solution().items()},
                                                       tableau=rb.tableau_reference().tolist())))
            else:
                self._envoyer(404, json.dumps(dict(erreur="introuvable")))

        def do_POST(self):
            u = urlparse(self.path)
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except Exception:
                self._envoyer(400, json.dumps(dict(erreur="JSON invalide"))); return
            try:
                if u.path == "/api/pose":
                    F = rb.fk(d.get("q", [0] * rb.n))
                    self._envoyer(200, json.dumps(dict(F=[f.ravel().tolist() for f in F])))
                elif u.path == "/api/construire":
                    res = rb.construire(_reperes_depuis_json(d.get("params")), d.get("tol_pos", 10.0), d.get("tol_ang", 6.0))
                    self._envoyer(200, json.dumps(res))
                elif u.path == "/api/analyze":
                    res = rb.analyser(_reperes_depuis_json(d.get("reperes")), d.get("tol_pos", 10.0), d.get("tol_ang", 6.0))
                    self._envoyer(200, json.dumps(res))
                elif u.path == "/api/export":
                    res = rb.analyser(_reperes_depuis_json(d.get("reperes")), d.get("tol_pos", 10.0), d.get("tol_ang", 6.0))
                    if not res["ok"]:
                        self._envoyer(409, json.dumps(dict(erreur="Les repères ne sont pas tous valides : corrigez-les avant d'exporter.")))
                    else:
                        fmt = d.get("fmt", "csv")
                        self._envoyer(200, json.dumps(dict(texte=formater_tableau(res["tableau"], fmt, rb.nom, rb))))
                else:
                    self._envoyer(404, json.dumps(dict(erreur="introuvable")))
            except Exception as e:
                self._envoyer(500, json.dumps(dict(erreur=f"{type(e).__name__}: {e}")))
    return H


def servir(rb, port=8765, ouvrir=True, aide_max=3, solution=False):
    page = PAGE
    srv = None
    for p in range(port, port + 20):
        try:
            srv = ThreadingHTTPServer(("127.0.0.1", p), faire_handler(rb, aide_max, solution, page))
            port = p
            break
        except OSError:
            continue
    if srv is None:
        raise RuntimeError("Aucun port libre.")
    url = f"http://127.0.0.1:{port}/"
    print(f"Atelier DH — {rb.nom} ({rb.n} articulations) : {url}\nCtrl+C pour arrêter.")
    if ouvrir:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nArrêt.")
    return srv


# ============================================================================
#  5. Ligne de commande
# ============================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description="Atelier Denavit-Hartenberg (MIRA)")
    sp = ap.add_subparsers(dest="cmd", required=True)

    def commun(p):
        p.add_argument("urdf")
        p.add_argument("--tip", help="lien terminal de la chaîne (si l'URDF est ramifié)")
        p.add_argument("--base", help="lien racine")

    s = sp.add_parser("serve", help="application locale dans le navigateur")
    commun(s)
    s.add_argument("--port", type=int, default=8765)
    s.add_argument("--no-open", action="store_true")
    s.add_argument("--solution", action="store_true", help="mode enseignant : active la solution complète")
    s.add_argument("--aide", type=int, default=2, choices=[1, 2, 3], help="niveau d'aide maximal (3 = réponse) ; défaut 2, 3 avec --solution")
    s.add_argument("--faces", type=int, default=5000, help="nombre maximal de faces par segment")
    t = sp.add_parser("table", help="tableau DH de référence")
    commun(t)
    t.add_argument("--format", default="text", choices=["text", "csv", "json", "tex"])
    c = sp.add_parser("check", help="vérifie des repères (fichier JSON {\"1\": {\"O\": [...], \"x\": [...]}, ...})")
    commun(c)
    c.add_argument("reperes")
    v = sp.add_parser("valider", help="valide un tableau DH (CSV/JSON) contre l'URDF")
    commun(v)
    v.add_argument("tableau")
    a = ap.parse_args(argv)
    if not os.path.isfile(a.urdf):
        sys.exit(f"Fichier URDF introuvable : {a.urdf}\n(dossier courant : {os.getcwd()}). "
                 f"Si vous avez téléchargé l'archive, décompressez-la puis lancez la commande depuis le dossier dh_studio.")
    avec_meshes = a.cmd == "serve"
    rb = Robot(a.urdf, tip=a.tip, base=a.base, mesh_faces=getattr(a, "faces", 5000), avec_meshes=avec_meshes)
    if a.cmd == "serve":
        servir(rb, a.port, not a.no_open, 3 if a.solution else a.aide, a.solution)
    elif a.cmd == "table":
        print(formater_tableau(rb.tableau_reference(), a.format, rb.nom, rb))
    elif a.cmd == "check":
        res = rb.analyser(_reperes_depuis_json(json.load(open(a.reperes, encoding="utf-8"))))
        for e in res["etats"]:
            sym = {"ok": "✔", "warn": "⚠", "err": "✘", "todo": "…"}[e["statut"]]
            p = e["params"]
            print(f"{sym} Repère {e['i']}" + (f" → θ0={p[0]:+.2f}° d={p[1]:.2f} r={p[2]:.2f} α={p[3]:+.2f}°" if p and e["statut"] in ("ok", "warn") else ""))
            for m in e["msgs"]:
                print("    ", m)
        sys.exit(0 if res["ok"] else 1)
    elif a.cmd == "valider":
        txt = open(a.tableau, encoding="utf-8").read()
        if a.tableau.endswith(".json"):
            d = json.loads(txt)
            T = [[l["theta0"], l["d"], l["r"], l["alpha"]] for l in d["lignes"]]
        else:
            T = [[float(x) for x in l.split(",")[1:5]] for l in txt.strip().splitlines()[1:]]
        ok, rap = rb.valider_tableau(T)
        print("\n".join(rap))
        sys.exit(0 if ok else 1)


# ============================================================================
#  6. Interface web
# ============================================================================
PAGE = r"""<!doctype html>
<html lang="fr"><head><meta charset="utf-8"><title>Atelier DH — __TITRE__</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--tq:#179AAF;--an:#363636;--ja:#FFCF00;--gr:#f3f5f7;--bd:#d8dde2}
*{box-sizing:border-box}body{margin:0;font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;color:var(--an);display:flex;height:100vh;overflow:hidden}
#vue{flex:1;min-width:0;position:relative}#plot{width:100%;height:100%}
#panel{width:420px;border-left:1px solid var(--bd);overflow-y:auto;padding:12px 14px;background:#fff}
h1{font-size:16px;margin:0 0 2px;color:var(--tq)}h2{font-size:13px;text-transform:uppercase;letter-spacing:.04em;margin:16px 0 6px;color:#666}
.sub{color:#777;font-size:12px;margin-bottom:8px}
.bar{display:flex;flex-wrap:wrap;gap:4px}
button{font:inherit;border:1px solid var(--bd);background:var(--gr);border-radius:6px;padding:4px 9px;cursor:pointer}
button:hover{border-color:var(--tq)}button.on{background:var(--tq);color:#fff;border-color:var(--tq)}
button.f{min-width:38px;font-weight:600}button.f.ok{border-color:#2e9d4f;color:#2e9d4f}button.f.err{border-color:#d33;color:#d33}button.f.warn{border-color:#d9a400;color:#a67c00}
button.f.cur{background:var(--an);color:#fff;border-color:var(--an)}
.row{display:flex;gap:6px;align-items:center;margin:4px 0}.row label{width:70px;color:#555}
input[type=number]{width:78px;padding:3px 5px;border:1px solid var(--bd);border-radius:5px;font:inherit}
input[type=range]{flex:1}
.log{font-size:12.5px;margin:6px 0}.log div{padding:2px 0}.ok{color:#2e9d4f}.err{color:#d33}.warn{color:#a67c00}.todo{color:#888}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}th,td{border:1px solid var(--bd);padding:3px 6px;text-align:right}th{background:var(--gr);font-weight:600}
.pre{white-space:pre-wrap;background:var(--gr);border-radius:6px;padding:8px;font-size:12.5px}
.hint{font-size:12px;color:#666}.badge{display:inline-block;padding:1px 7px;border-radius:9px;font-size:11px;background:var(--ja);color:#000;margin-left:6px}
#edit .box{border:1px solid var(--bd);border-radius:8px;padding:6px 9px;margin:6px 0}#edit .bt{font-weight:600;margin-bottom:2px}
#pick{position:absolute;left:10px;top:8px;background:rgba(255,255,255,.92);border:1px solid var(--bd);border-radius:6px;padding:4px 9px;font-size:12.5px;display:none}
</style></head><body>
<div id="vue"><div id="plot"></div></div>
<div id="panel">
 <h1>Atelier DH <span id="nom"></span></h1><div class="sub">Convention standard : A<sub>i</sub> = Rot(z,θ) Trans(z,d) Trans(x,r) Rot(x,α) · mm et degrés</div>

 <h2>Affichage</h2>
 <div class="bar"><button id="bAxes" class="on">Axes articulaires</button><button id="bFr" class="on">Repères</button><button id="bPng">Image PNG</button></div>
 <div class="row"><label>Opacité</label><input type="range" id="opac" min="0.1" max="1" step="0.05" value="0.45"></div>
 <details><summary>Explorer le robot (articulations)</summary><div id="sliders"></div><div class="bar"><button id="bZero">q = 0</button></div>
  <div class="hint">Les repères sont définis robot à q = 0 ; en bougeant, ils suivent les segments. Le bouton « q = 0 » ramène le robot en position de construction.</div></details>

<h2>Repères</h2>
 <div class="sub" id="rep0"></div>
 <div class="bar" id="fbar"></div>
 <div id="edit">
  <div class="box"><div class="bt">Origine <span id="tO"></span></div>
   <div id="oForce" class="hint" style="display:none"></div>
   <div id="oLibre"><div class="row"><input type="range" id="ts" min="-500" max="500" step="0.5" value="0"><input type="number" id="tn" step="0.1" style="width:78px"> mm</div>
    <div class="hint">Faites glisser pour déplacer O<sub id="ti">i</sub> le long de l'axe z<sub id="ti2">i</sub> (jaune) ; la position est mesurée depuis le point de référence de l'axe.</div></div>
   <div class="hint" id="oCoord"></div></div>
  <div class="box"><div class="bt">Direction de x <span id="tX"></span></div>
   <div id="xAttente" class="hint" style="display:none">Placez d'abord l'origine.</div>
   <div id="xForce" style="display:none"><div class="hint" id="xForceTxt"></div><div class="bar"><button id="xSign">Inverser le sens de x</button></div></div>
   <div id="xLibre" style="display:none"><div class="row"><input type="range" id="ps" min="-180" max="180" step="1" value="0"><input type="number" id="pn" step="0.5" style="width:70px"> °</div>
    <div class="hint">Angle de x<sub class="xi">i</sub> par rapport à x<sub class="xp">i-1</sub>, autour de z<sub class="zp">i-1</sub> (x<sub class="xi">i</sub> reste perpendiculaire à z<sub class="zp">i-1</sub>).</div></div></div>
  <div class="bar"><button id="vide">Réinitialiser ce repère</button></div>
 </div>
 <div class="row"><label>Tolérance</label><input type="number" id="tpos" value="10" step="1" style="width:56px"> mm <input type="number" id="tang" value="6" step="1" style="width:46px"> °</div>
 <div class="log" id="log"></div>

 <h2>Tableau DH</h2>
 <table id="tab"><thead><tr><th>i</th><th>θ₀ (°)</th><th>d (mm)</th><th>r (mm)</th><th>α (°)</th></tr></thead><tbody></tbody></table>
 <div class="bar" style="margin-top:6px"><button data-ex="csv">CSV</button><button data-ex="json">JSON</button><button data-ex="tex">LaTeX</button><button id="cp">Copier</button></div>

 <h2>Aide</h2>
 <div class="bar" id="abar"></div><div class="pre" id="aide" style="margin-top:6px;display:none"></div>
 <div id="prof" style="display:none"><h2>Enseignant</h2><div class="bar"><button id="bSol">Charger la solution de référence</button></div></div>
</div>
<script src="/plotly.min.js"></script>
<script>
const $=id=>document.getElementById(id);
let R=null,F=null,q=[],cur=1,analyse=null,showAx=true,showFr=true,showNorm=false,opac=.45,ready=false,timer=null;
const f2=v=>Math.round(v*100)/100;

// ---------- algèbre 4x4 (ligne par ligne)
const M4=a=>a;function mul(A,B){const C=new Array(16).fill(0);for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;for(let k=0;k<4;k++)s+=A[i*4+k]*B[k*4+j];C[i*4+j]=s}return C}
function invRigid(A){const R=[A[0],A[4],A[8],A[1],A[5],A[9],A[2],A[6],A[10]];const t=[A[3],A[7],A[11]];
 const ti=[-(R[0]*t[0]+R[1]*t[1]+R[2]*t[2]),-(R[3]*t[0]+R[4]*t[1]+R[5]*t[2]),-(R[6]*t[0]+R[7]*t[1]+R[8]*t[2])];
 return [R[0],R[1],R[2],ti[0],R[3],R[4],R[5],ti[1],R[6],R[7],R[8],ti[2],0,0,0,1]}
function ap(A,p){return[A[0]*p[0]+A[1]*p[1]+A[2]*p[2]+A[3],A[4]*p[0]+A[5]*p[1]+A[6]*p[2]+A[7],A[8]*p[0]+A[9]*p[1]+A[10]*p[2]+A[11]]}
function apv(A,v){return[A[0]*v[0]+A[1]*v[1]+A[2]*v[2],A[4]*v[0]+A[5]*v[1]+A[6]*v[2],A[8]*v[0]+A[9]*v[1]+A[10]*v[2]]}
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]],dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],nrm=a=>Math.hypot(a[0],a[1],a[2]);
const unit=a=>{const n=nrm(a)||1;return[a[0]/n,a[1]/n,a[2]/n]};

async function get(u){return (await fetch(u)).json()}
async function post(u,d){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});return {ok:r.ok,d:await r.json()}}

// ---------- tracés
let meshIdx=[],dynStart=0;
function meshTraces(){return R.liens.map((l,m)=>{const V=l.verts,n=V.length/3,x=new Array(n),y=new Array(n),z=new Array(n),T=F[m];
 for(let i=0;i<n;i++){const p=ap(T,[V[3*i],V[3*i+1],V[3*i+2]]);x[i]=p[0];y[i]=p[1];z[i]=p[2]}
 const f=l.faces,nf=f.length/3,I=new Array(nf),J=new Array(nf),K=new Array(nf);for(let i=0;i<nf;i++){I[i]=f[3*i];J[i]=f[3*i+1];K[i]=f[3*i+2]}
 return {type:'mesh3d',x,y,z,i:I,j:J,k:K,color:l.couleur,opacity:opac,flatshading:false,lighting:{ambient:.62,diffuse:.65,specular:.25,roughness:.5,fresnel:.2},lightposition:{x:800,y:-1200,z:1500},name:'segment '+m,showlegend:false,
 hovertemplate:'segment '+m+'<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f} mm<extra></extra>',meta:'mesh'}})}
function axeTraces(){const L=R.taille*0.55,out=[];R.axes.slice(0,R.n).forEach((a,k)=>{const xs=[],ys=[],zs=[],P=ap(F[k],[0,0,0]);
 // l'axe k est fixe dans le segment k : on le déplace avec F[k]F0[k]^-1
 const M=mul(F[k],invRigid(R.F0m[k]));const p0=ap(M,a.p),u=apv(M,a.u);
 for(let t=-L;t<=L;t+=L/20){xs.push(p0[0]+t*u[0]);ys.push(p0[1]+t*u[1]);zs.push(p0[2]+t*u[2])}
 out.push({type:'scatter3d',mode:'lines',x:xs,y:ys,z:zs,line:{color:'#FFCF00',width:5,dash:'dash'},showlegend:false,hovertemplate:'axe de l\'articulation '+(k+1)+'<br>x=%{x:.1f} y=%{y:.1f} z=%{z:.1f}<extra></extra>',meta:'axe'});
 const e=[p0[0]+L*u[0],p0[1]+L*u[1],p0[2]+L*u[2]];out.push({type:'scatter3d',mode:'text',x:[e[0]],y:[e[1]],z:[e[2]],text:[(R.joints[k].type=='P'?'prism. ':'art. ')+(k+1)],textfont:{color:'#b58f00',size:12},showlegend:false,hoverinfo:'skip',meta:'axe'})});return out}
function frameTraces(){const out=[];if(!analyse)return out;const L=R.taille*0.1;
 analyse.reperes.forEach((r,k)=>{const M=mul(F[k],invRigid(R.F0m[k]));const O=ap(M,r.O),x=apv(M,r.x),z=apv(M,r.z),y=cross(z,x);
  [[x,'#d62728','x'],[y,'#2ca02c','y'],[z,'#1f4fd6','z']].forEach(([v,c,l])=>{out.push({type:'scatter3d',mode:'lines',x:[O[0],O[0]+L*v[0]],y:[O[1],O[1]+L*v[1]],z:[O[2],O[2]+L*v[2]],line:{color:c,width:6},showlegend:false,hoverinfo:'skip',meta:'rep'});
   out.push({type:'scatter3d',mode:'text',x:[O[0]+1.2*L*v[0]],y:[O[1]+1.2*L*v[1]],z:[O[2]+1.2*L*v[2]],text:[l+'<sub>'+k+'</sub>'],textfont:{color:c,size:13},showlegend:false,hoverinfo:'skip',meta:'rep'})});
  out.push({type:'scatter3d',mode:'markers',x:[O[0]],y:[O[1]],z:[O[2]],marker:{size:4,color:'#363636'},showlegend:false,hovertemplate:'O<sub>'+k+'</sub> : %{x:.1f} %{y:.1f} %{z:.1f} mm<extra></extra>',meta:'rep'})});
 if(showNorm&&R.solution_ok&&R.normales){R.normales.forEach(s=>out.push({type:'scatter3d',mode:'lines',x:[s[0][0],s[1][0]],y:[s[0][1],s[1][1]],z:[s[0][2],s[1][2]],line:{color:'#e377c2',width:8},showlegend:false,hoverinfo:'skip',meta:'rep'}))}
 return out}
function layout(){const ax={showbackground:false,gridcolor:'#e2e2e2',zerolinecolor:'#999'};return{margin:{l:0,r:0,t:0,b:0},paper_bgcolor:'white',uirevision:'cam',
 scene:{xaxis:{...ax,title:'x (mm)'},yaxis:{...ax,title:'y (mm)'},zaxis:{...ax,title:'z (mm)'},aspectmode:'data',camera:{eye:{x:1.4,y:-1.6,z:.8}}}}}
async function draw(full){
 const dyn=[...(showAx?axeTraces():[]),...(showFr?frameTraces():[])];
 if(full){await Plotly.react('plot',[...meshTraces(),...dyn],layout(),{responsive:true,displaylogo:false});bindClick();}
 else{const gd=$('plot');const n=gd.data.length;const nm=R.liens.length;if(n>nm)await Plotly.deleteTraces(gd,[...Array(n-nm).keys()].map(i=>i+nm));if(dyn.length)await Plotly.addTraces(gd,dyn)}
}
function bindClick(){}

// ---------- repères : édition par curseurs
let P={},meta=null;     // P[i] = {t, sign, phi}
function sauver(){try{localStorage.setItem('dh2_'+R.nom,JSON.stringify(P))}catch(e){}}
function charger(){try{const s=localStorage.getItem('dh2_'+R.nom);if(s)P=JSON.parse(s)}catch(e){}}
function planifier(){clearTimeout(timer);timer=setTimeout(construire,40)}
const sub_i=(c,t)=>document.querySelectorAll('.'+c).forEach(e=>e.textContent=t);
async function construire(){
 const {d}=await post('/api/construire',{params:P,tol_pos:parseFloat($('tpos').value)||10,tol_ang:parseFloat($('tang').value)||6});
 meta=d.frames;analyse=d.analyse;sauver();rendreEdition();rendreResultats();await draw(false)}
function rendreEdition(){
 const fr=meta[cur-1],p=P[cur]||{};$('ti').textContent=cur;$('ti2').textContent=cur;$('tO').textContent='O'+cur;$('tX').textContent='x'+cur;
 sub_i('xi',cur);sub_i('xp',cur-1);sub_i('zp',cur-1);
 const o=fr.origine;
 if(o.mode==='force'){$('oForce').style.display='block';$('oLibre').style.display='none';
  $('oForce').innerHTML='<b>O'+cur+' est imposé</b> : les axes z'+(cur-1)+' et z'+cur+' sont sécants, l\'origine est leur point d\'intersection.'}
 else{$('oForce').style.display='none';$('oLibre').style.display='block';
  const ts=$('ts'),tn=$('tn');ts.min=o.tmin;ts.max=o.tmax;const v=(p.t!==undefined&&p.t!==null)?p.t:0;
  if(document.activeElement!==ts)ts.value=v;if(document.activeElement!==tn)tn.value=Math.round(v*10)/10}
 $('oCoord').textContent=fr.O?'O'+cur+' = ('+fr.O.map(f2).join(', ')+') mm':(o.mode==='force'?'':'O'+cur+' : pas encore placé (déplacez le curseur).');
 const x=fr.x||{};$('xAttente').style.display=$('xForce').style.display=$('xLibre').style.display='none';
 if(fr.bloque||fr.O===null){$('xAttente').style.display='block';$('xAttente').textContent=fr.bloque?'En attente des repères précédents.':"Placez d'abord l'origine O"+cur+'.'}
 else if(x.mode==='force'){$('xForce').style.display='block';
  $('xForceTxt').innerHTML='<b>x'+cur+' est imposé</b> (au sens près) : '+(x.nature==='sécants'?'perpendiculaire à z'+(cur-1)+' et à z'+cur+' (axes sécants).':'il doit couper z'+(cur-1)+' et rester perpendiculaire à z'+(cur-1)+' : c\'est la direction qui joint z'+(cur-1)+' à O'+cur+'.')+
   (x.sign<0?' <i>(sens inversé)</i>':'')}
 else if(x.mode==='libre'){$('xLibre').style.display='block';const ps=$('ps'),pn=$('pn'),v=x.phi||0;if(document.activeElement!==ps)ps.value=v;if(document.activeElement!==pn)pn.value=Math.round(v*10)/10}
 document.querySelectorAll('#fbar button').forEach(b=>b.classList.toggle('cur',+b.dataset.i===cur))}
function rendreResultats(){const d=analyse;
 const lg=$('log');lg.innerHTML='';d.etats.forEach(e=>{const sym={ok:'✔',warn:'⚠',err:'✘',todo:'…'}[e.statut];let h='<b>'+sym+' Repère '+e.i+'</b>';
  if(e.params&&(e.statut=='ok'||e.statut=='warn'))h+=R.joints[e.i-1].type=='P'?' → θ='+f2(e.params[0])+'° d₀='+f2(e.params[1])+' (variable : d = d₀ + q) r='+f2(e.params[2])+' α='+f2(e.params[3])+'°':' → θ₀='+f2(e.params[0])+'° d='+f2(e.params[1])+' r='+f2(e.params[2])+' α='+f2(e.params[3])+'°';
  const div=document.createElement('div');div.className=e.statut;div.innerHTML=h+e.msgs.map(m=>'<div style="margin-left:14px;font-weight:normal">'+m+'</div>').join('');lg.appendChild(div)});
 if(d.ok){const div=document.createElement('div');div.className='ok';div.innerHTML='<b>Tous les repères respectent les règles DH.</b>';lg.appendChild(div)}
 const tb=document.querySelector('#tab tbody');tb.innerHTML='';for(let i=0;i<R.n;i++){const p=d.etats[i].params,ok=d.etats[i].statut=='ok'||d.etats[i].statut=='warn';
  tb.innerHTML+='<tr><td>'+(i+1)+'</td>'+(p&&ok?p.map((v,c)=>'<td'+(c==(R.joints[i].type=='P'?1:0)?' style="background:#fff4c2;font-weight:600"':'')+'>'+f2(v)+'</td>').join(''):'<td colspan=4 style="text-align:center;color:#aaa">—</td>')+'</tr>'}
 d.etats.forEach(e=>{const b=document.querySelector('#fbar button[data-i="'+e.i+'"]');b.className='f '+(e.statut=='todo'?'':e.statut)+(e.i==cur?' cur':'')})}
function choisir(i){cur=i;if(meta)rendreEdition()}
function majP(champ,val){P[cur]=Object.assign({},P[cur]||{},{[champ]:val});planifier()}

// ---------- init
(async()=>{
 R=await get('/api/robot');F=R.F0.map(a=>a.slice());R.F0m=R.F0;q=Array(R.n).fill(0);$('nom').textContent='— '+R.nom;
 const r0=R.rep0;$('rep0').innerHTML='Repère 0 imposé : O₀ = ('+r0.O.map(f2).join(', ')+') mm · x₀ = ('+r0.x.map(v=>f2(v)).join(', ')+') · z₀ = axe de l\'articulation 1';
 const fb=$('fbar');for(let i=1;i<=R.n;i++){const b=document.createElement('button');b.className='f'+(i==1?' cur':'');b.textContent=i;b.dataset.i=i;b.onclick=()=>choisir(i);fb.appendChild(b)}
 const ab=$('abar');for(let l=1;l<=R.aide_max;l++){const b=document.createElement('button');b.textContent='Aide niveau '+l;b.onclick=async()=>{const {texte}=await get('/api/aide?i='+cur+'&niveau='+l);const e=$('aide');e.style.display='block';e.textContent=texte};ab.appendChild(b)}
 if(R.aide_max<3){const s=document.createElement('span');s.className='hint';s.textContent=' (la réponse complète est désactivée)';ab.appendChild(s)}
 const sl=$('sliders');R.joints.forEach((j,k)=>{const d=document.createElement('div');d.className='row';const P=j.type=='P',un=P?' mm':'°',mn=Math.round(P?j.lo:Math.max(j.lo,-180)),mx=Math.round(P?j.hi:Math.min(j.hi,180)),v0=Math.min(Math.max(0,mn),mx);d.innerHTML='<label>'+(P?'d':'q')+(k+1)+'</label><input type="range" min="'+mn+'" max="'+mx+'" step="1" value="'+v0+'" data-k="'+k+'" data-un="'+un+'"><span style="width:58px;text-align:right" id="qv'+k+'">'+v0+un+'</span>';sl.appendChild(d)});
 sl.querySelectorAll('input').forEach(inp=>inp.oninput=async()=>{const k=+inp.dataset.k;q[k]=+inp.value;$('qv'+k).textContent=q[k]+inp.dataset.un;const {d}=await post('/api/pose',{q});F=d.F;
  const gd=$('plot');const upd=meshTraces();Plotly.restyle(gd,{x:upd.map(t=>t.x),y:upd.map(t=>t.y),z:upd.map(t=>t.z)},[...Array(R.liens.length).keys()]);await draw(false)});
 $('bZero').onclick=async()=>{q=Array(R.n).fill(0);sl.querySelectorAll('input').forEach(i=>{i.value=Math.min(Math.max(0,+i.min),+i.max);$('qv'+i.dataset.k).textContent=i.value+i.dataset.un});F=R.F0.map(a=>a.slice());await draw(true);await construire()};
  if(R.solution){$('prof').style.display='block';$('bSol').onclick=async()=>{const s=await get('/api/solution');P={};Object.keys(s.params).forEach(k=>P[k]={t:s.params[k].t,sign:s.params[k].sign,phi:s.params[k].phi});construire()}}
 $('ts').oninput=e=>{$('tn').value=Math.round(+e.target.value*10)/10;majP('t',+e.target.value)};
 $('tn').oninput=e=>{if(e.target.value==='')return;$('ts').value=e.target.value;majP('t',+e.target.value)};
 $('ps').oninput=e=>{$('pn').value=e.target.value;majP('phi',+e.target.value)};
 $('pn').oninput=e=>{if(e.target.value==='')return;$('ps').value=e.target.value;majP('phi',+e.target.value)};
 $('xSign').onclick=()=>{const s=(P[cur]&&P[cur].sign===-1)?1:-1;majP('sign',s)};
 $('vide').onclick=()=>{delete P[cur];planifier()};
 $('bAxes').onclick=e=>{showAx=!showAx;e.target.classList.toggle('on',showAx);draw(true)};$('bFr').onclick=e=>{showFr=!showFr;e.target.classList.toggle('on',showFr);draw(true)};
  $('opac').oninput=e=>{opac=+e.target.value;Plotly.restyle('plot',{opacity:opac},[...Array(R.liens.length).keys()])};
 $('bPng').onclick=()=>Plotly.downloadImage('plot',{format:'png',filename:'robot_reperes_DH',width:1400,height:900});
 $('tpos').onchange=$('tang').onchange=planifier;
 document.querySelectorAll('[data-ex]').forEach(b=>b.onclick=async()=>{const cl={};for(let i=1;i<=R.n;i++){const r=analyse.reperes[i];cl[i]=r?{O:r.O,x:r.x}:{O:null,x:null}}
  const {ok,d}=await post('/api/export',{reperes:cl,fmt:b.dataset.ex,tol_pos:parseFloat($('tpos').value)||10,tol_ang:parseFloat($('tang').value)||6});
  if(!ok){alert(d.erreur);return}const blob=new Blob([d.texte],{type:'text/plain'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='tableau_DH.'+b.dataset.ex.replace('tex','tex');a.click()});
 $('cp').onclick=async()=>{const cl={};for(let i=1;i<=R.n;i++){const r=analyse.reperes[i];cl[i]=r?{O:r.O,x:r.x}:{O:null,x:null}}const {ok,d}=await post('/api/export',{reperes:cl,fmt:'csv'});if(!ok){alert(d.erreur);return}try{await navigator.clipboard.writeText(d.texte);$('cp').textContent='Copié ✔'}catch(e){prompt('Copiez :',d.texte)}setTimeout(()=>$('cp').textContent='Copier',1500)};
 charger();await draw(true);await construire();ready=true;window.__ready=true;
 window.__dh={P:()=>P,setP:(p)=>{P=p;return construire()},setCur:choisir,analyse:()=>analyse,meta:()=>meta};
})();
</script></body></html>
"""

if __name__ == "__main__":
    main()
