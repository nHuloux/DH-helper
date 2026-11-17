# Atelier DH — repères de Denavit-Hartenberg sur n'importe quel robot sériel

Site **statique** (HTML + JavaScript, aucun serveur) : on dépose le fichier URDF d'un robot (avec ses maillages) et on place les repères de Denavit-Hartenberg (convention **standard**) à l'aide de curseurs, avec vérification des règles DH, tableau `θ₀, d, r, α` et exports CSV / JSON / LaTeX / PNG. Il peut aussi **valider le tableau DH d'un étudiant** contre la cinématique de l'URDF. Les fichiers déposés restent dans le navigateur de l'utilisateur.

Convention : `A_i = Rot(z,θ_i) · Trans(z,d_i) · Trans(x,r_i) · Rot(x,α_i)`, `θ_i = θ0_i + q_i`, millimètres et degrés. Repères définis robot à q = 0.

## Publier sur GitHub Pages

1. Créer un dépôt GitHub (ex. `atelier-dh`) et y envoyer **le contenu de ce dossier** (le fichier `index.html` doit être à la racine du dépôt) :
   ```
   git init && git add . && git commit -m "Atelier DH"
   git branch -M main
   git remote add origin https://github.com/<organisation>/atelier-dh.git
   git push -u origin main
   ```
2. Dans le dépôt : **Settings → Pages → Build and deployment → Source : GitHub Actions** (le workflow `.github/workflows/pages.yml` est déjà fourni). Variante sans workflow : *Source : Deploy from a branch → main / (root)*.
3. Après quelques dizaines de secondes, le site est en ligne sur `https://<organisation>.github.io/atelier-dh/`.

Le fichier `.nojekyll` (à conserver) évite que GitHub Pages ne retraite les fichiers. Aucune étape de compilation : les bibliothèques (three.js, JSZip) sont fournies dans `vendor/`, le site fonctionne donc aussi hors ligne.

## Utilisation

- **Robots d'exemple** : xArm 6 (robot du cours), xArm 7, Lite 6, UR5 simplifié.
- **Votre robot** : glisser-déposer le dossier du robot (fichier `.urdf` + maillages `.stl`, `.dae`, `.obj`, `.glb`) ou une archive `.zip` ; ou boutons « Choisir un dossier / un .zip ». Les chemins de maillages `package://…`, `file://…` ou relatifs sont retrouvés par leur fin de chemin.
- Chaîne **sérielle à articulations rotoïdes**. Si l'URDF est ramifié, un menu permet de choisir la chaîne (lien terminal). Les articulations prismatiques situées après la dernière rotoïde sont ignorées ; une prismatique au milieu de la chaîne est refusée. Les fichiers **xacro** doivent d'abord être convertis (`xacro robot.urdf.xacro > robot.urdf`).
- Pour chaque repère i : l'origine O_i glisse le long de l'axe z_i (imposée si z_{i-1} et z_i sont sécants) ; la direction de x_i est imposée (au sens près) sauf cas libre où un curseur d'angle apparaît. L'outil signale la règle violée (x ⟂ z_{i-1}, x coupe z_{i-1}, normale commune, origine sur l'axe…).
- Section **Vérifier un tableau DH** : coller les lignes `θ₀ d r α` d'un étudiant ; le test compare, pour des configurations aléatoires, ses repères à ceux de l'URDF et indique la première ligne suspecte.

### Paramètres d'URL

| Paramètre | Effet |
|---|---|
| `?robot=xarm6` | ouvre directement un exemple (`xarm6`, `xarm7`, `lite6`, `ur5`) |
| `?aide=0..3` | niveau d'aide maximal (0 = aucune aide ; 2 = sans la réponse ; 3 = tout, par défaut) |
| `?prof=1` | affiche le bouton « Charger la solution de référence » |

Exemple pour une séance : `https://…github.io/atelier-dh/?robot=xarm6&aide=2`. **Attention** : ces paramètres ne sont pas une protection (le site est entièrement côté client, un étudiant peut les modifier) ; ils servent à cadrer une séance, pas à garder la solution secrète.

## Ajouter un robot d'exemple

Déposer un dossier dans `robots/<id>/` (URDF + maillages), puis ajouter une entrée dans `robots/index.json` (`id`, `nom`, `description`, `dossier`, liste `fichiers` relative au dossier). Compter < 2,5 Mo de maillages par robot (STL binaire décimés) pour un chargement rapide.

## Organisation

```
index.html            page unique
css/style.css
js/linalg.js          algèbre linéaire
js/robot.js           lecture URDF, axes, repères DH, analyse, validation, exports (aucune dépendance)
js/meshes.js          lecture STL / OBJ / DAE / GLB
js/files.js           dossier déposé, zip, exemples
js/viewer.js          scène three.js (Z vers le haut, mm)
js/app.js             interface
robots/               exemples (+ index.json)
vendor/               three.js (MIT), JSZip (MIT/GPLv3)
outils_python/        version locale Python équivalente (dh_studio.py v3, mêmes règles)
.github/workflows/    déploiement GitHub Pages
```

`js/robot.js` est un portage fidèle du moteur Python `dh_studio.py` ; les deux ont été comparés sur xArm 6, xArm 7, Lite 6 et UR5 (tableau de référence, analyse de repères corrects et erronés, construction par curseurs, validation de tableaux, exports) avec des écarts inférieurs à 10⁻⁶.

## Limites

- Convention DH **standard** uniquement (pas de DH modifiée) ; rotoïdes uniquement.
- Les repères sont calculés à partir des axes de l'URDF : un URDF dont les angles sont arrondis (ex. `1.5708`) est toléré (axes parallèles à 0,1° près, sécants à 0,1 mm près).
- Navigateurs récents (Chrome, Edge, Firefox, Safari) avec WebGL. Le dépôt d'un **dossier** par glisser-déposer fonctionne sur les principaux navigateurs de bureau ; à défaut, utiliser « Choisir un dossier » ou un `.zip`.

## Licences et crédits

- Atelier DH : MIRA — École d'ingénieur·e·s (Groupe Aflokkat).
- three.js © les auteurs de three.js, licence MIT (`vendor/three/LICENSE`) ; JSZip, licence MIT (`vendor/jszip/LICENSE.markdown`).
- Maillages et cinématique des xArm 6 / xArm 7 / Lite 6 : UFACTORY xarm_ros, licence BSD (`robots/LICENSE-xarm_ros.txt`), maillages décimés. UR5 : modèle simplifié écrit pour l'outil.
