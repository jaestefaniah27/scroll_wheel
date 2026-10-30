# scroll_wheel (espejo de jaestefaniah27/Orvy)

El código se trabaja en el monorepo Orvy; este repo guarda además las releases de la app y del firmware.
## Releases y etiquetas: SIEMPRE desde el servidor

Las sesiones de Claude en la nube **no pueden** crear etiquetas ni releases en GitHub (el proxy devuelve 403 a
`git push` de etiquetas y a la API de releases). No lo intentes desde la sesión ni pidas al usuario que lo haga a
mano: se hace desde el servidor Oracle con el conector **Authorization** (`mcp__Authorization__run_command`,
usuario `ubuntu`).

- Las releases de app (`v*`) y de firmware (`fw-v*`) viven en **jaestefaniah27/scroll_wheel**, no en Orvy.
- Clon para publicar: `~/orvy-release/scroll_wheel` en ese servidor. Su `git push` va por SSH con la clave de la
  cuenta (`~/.ssh/github_poker`): sirve para las etiquetas.
- Crear la release y subir el `.uf2` va por la API de GitHub con el token que **ya está** en el servidor:
  `/etc/orvy/github-release.token` (de root, 600; `ubuntu` lo lee con `sudo`). No lo pidas ni lo imprimas: el
  script lo lee solo. Buscarlo solo en `~` no lo encuentra.
- Pasos: (1) compilar el `.uf2` en la sesión y subirlo, con su `.sha256` y `notas.md`, a una rama temporal
  `release-assets/<etiqueta>` de scroll_wheel (las ramas sí se pueden subir desde la sesión); (2) en el servidor,
  traerla a `~/orvy-release/<etiqueta>/`, etiquetar y `git push` la etiqueta desde el clon; (3)
  `~/orvy-release/publicar-release.sh <etiqueta> "<título>" <notas.md> <1|0> <ficheros>` crea la release y sube
  los ficheros; (4) borrar la rama temporal.
- Firmware: siempre `--prerelease` / `"prerelease": true`; con `ORBY_V4-fw-X.Y.uf2` y su `.sha256` como assets.
  Detalle y motivo en `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/docs/COMPATIBILIDAD.md`.
