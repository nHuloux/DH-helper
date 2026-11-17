// xacro.js — conversion xacro → URDF dans le navigateur (bibliothèque xacro-parser, MIT, chargée seulement quand nécessaire).
// Les blocs inutiles pour la cinématique (ros2_control, gazebo, transmission) sont retirés avant traitement,
// car ils utilisent des fonctions que le moteur n'implémente pas (xacro.load_yaml, etc.).

export function simplifierXacro(texte) {
  return texte
    .replace(/<ros2_control\b[\s\S]*?<\/ros2_control>/g, '')
    .replace(/<gazebo\b[\s\S]*?<\/gazebo>/g, '')
    .replace(/<transmission\b[\s\S]*?<\/transmission>/g, '')
    .replace(/\$\{\s*xacro\.load_yaml\([^}]*\)\s*\}/g, '0');
}

// Arguments déclarés dans le fichier principal : <xacro:arg name="dof" default="7"/>
export function argumentsXacro(texte) {
  const res = [];
  const re = /<xacro:arg\b([^>]*?)\/?>/g;
  let m;
  while ((m = re.exec(texte))) {
    const nom = /name\s*=\s*"([^"]*)"/.exec(m[1]);
    const def = /default\s*=\s*"([^"]*)"/.exec(m[1]);
    if (nom) res.push({ nom: nom[1], defaut: def ? def[1] : '' });
  }
  return res;
}

// Un fichier xacro est « principal » s'il a une racine <robot> avec des arguments ou n'est inclus par personne ; heuristique simple :
export const ressembleAPrincipal = (texte) => /<robot\b/.test(texte);

/**
 * @param {object} o
 *  texte        contenu du xacro principal
 *  dossier      dossier du fichier principal (chemin dans le dépôt de fichiers)
 *  lire(chemin) → Promise<string>   contenu d'un fichier (chemin tel que construit par le parseur)
 *  paquet(nom)  → chemin du paquet (lève une erreur si inconnu)
 *  args         {nom: valeur}
 */
export async function xacroVersUrdf({ texte, dossier, lire, paquet, args = {} }) {
  const { XacroParser } = await import('../vendor/xacro/xacro-parser.js');
  const parser = new XacroParser();
  parser.arguments = args;
  parser.workingPath = dossier ? `${dossier}/` : '';
  parser.rospackCommands = {
    find: (n) => paquet(n),
    env: (n) => '',
    optenv: (n, d = '') => d,
  };
  parser.getFileContents = async (chemin) => simplifierXacro(await lire(chemin));
  const doc = await parser.parse(simplifierXacro(texte));
  return new XMLSerializer().serializeToString(doc);
}
