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
- Crear la release y subir el `.uf2` va por la API de GitHub y necesita un token con permiso de escritura en
  scroll_wheel, en `~/.config/github/token` del mismo servidor (fuera del repo, `chmod 600`).
- Firmware: siempre `--prerelease` / `"prerelease": true`; con `ORBY_V4-fw-X.Y.uf2` y su `.sha256` como assets.
  Detalle y motivo en `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/docs/COMPATIBILIDAD.md`.
