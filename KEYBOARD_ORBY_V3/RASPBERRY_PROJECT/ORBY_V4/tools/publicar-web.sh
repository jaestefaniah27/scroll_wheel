#!/usr/bin/env bash
# Publica la WebGUI (y la landing) en la rama gh-pages de scroll_wheel.
#
# Se lanza A MANO, solo cuando se pide. GitHub Actions no corre en esta cuenta, así
# que no hay publicación automática: Pages sirve la rama gh-pages tal cual y, si no
# se ejecuta esto, /app/ se queda en la última versión subida.
#
# Hace falta GH_TOKEN con permiso de escritura sobre el repo.
# Uso: tools/publicar-web.sh [mensaje del commit]
set -euo pipefail

PROYECTO="$(cd "$(dirname "$0")/.." && pwd)"
REPO="jaestefaniah27/scroll_wheel"
: "${GH_TOKEN:?falta GH_TOKEN}"

VERSION="$(node -p "require('$PROYECTO/OrbyGUI/package.json').version")"
MENSAJE="${1:-Actualiza la WebGUI y la landing a Orvy $VERSION}"

(cd "$PROYECTO/OrbyGUI" && npm run build)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

git clone -q --depth 1 -b gh-pages "https://x-access-token:$GH_TOKEN@github.com/$REPO.git" "$TMP/gp"
cd "$TMP/gp"
# Sin esto Git en Windows reescribe los finales de línea de los bundles.
git config core.autocrlf false

rm -rf app
mkdir app
cp -r "$PROYECTO/OrbyGUI/dist/." app/
cp "$PROYECTO/web/index.html" index.html

git add -A
if git diff --cached --quiet; then
  echo "gh-pages ya está al día, no hay nada que subir."
  exit 0
fi
git commit -q -m "$MENSAJE

Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>"
git push -q origin gh-pages
echo "Publicado: https://jaestefaniah27.github.io/scroll_wheel/app/ (Orvy $VERSION). Pages tarda ~1 min."
