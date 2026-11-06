# ALTO: el código ya no se trabaja aquí

**Desde el 6 de noviembre de 2026 el código de Orvy vive en `jaestefaniah27/Orvy` (privado).** Todo lo que
había en este repositorio —`main` con el Blackjack, Orvy 1.1.22, el firmware 4.33 y la rama `airstrike`—
está fusionado en `main` de Orvy. Ningún commit de código va ya a scroll_wheel.

Si eres un agente y estás leyendo esto en un clon de scroll_wheel:

1. **No hagas commits de código aquí, ni en `main` ni en ramas nuevas.** Para, y cambia el clon a Orvy:
   ```bash
   git remote set-url origin git@github.com:jaestefaniah27/Orvy.git
   git fetch origin
   git switch main && git reset --hard origin/main   # solo si no hay nada sin subir
   ```
   ¿Hay commits sin subir? Entonces `git switch -c <persona>/<issue>-<tema>` y `git merge origin/main`:
   `main` de Orvy tiene a `scroll_wheel/main` como antepasado y entra limpio.
2. **Lee `AGENTS.md` en la raíz de Orvy** antes de seguir: issues con dueño, ramas `<persona>/<issue>-<tema>`,
   reservas de versiones y relevo al cerrar sesión.
3. Si no tienes acceso a Orvy, no sigas: díselo a la persona con la que trabajas.

## Lo único que se sigue tocando aquí

La app instalada descarga cosas de este repositorio público, así que esto **no** se borra ni se mueve
todavía:

| Qué | Por qué sigue aquí |
|---|---|
| Releases `v*` y `fw-v*`, y ramas temporales `release-assets/*` | el actualizador lee `releases/latest/download/latest.json` y el firmware de las releases |
| `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/src/compat.json` en `main` | la app lo baja de `raw.githubusercontent.com/.../scroll_wheel/main/` al arrancar |
| `KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/tips-video/` en `main` | vídeos de los consejos, misma vía |
| Rama `gh-pages` | la WebGUI y la landing en `jaestefaniah27.github.io/scroll_wheel` |

Esos ficheros se cambian **primero en Orvy** y luego se copian tal cual aquí, en un commit que solo toque eso.
Las releases se siguen publicando aquí como cuenta `CLAUDE.md`, pero **compiladas desde Orvy**.

Cuando exista `orvy-releases` y el parque instalado se haya actualizado, scroll_wheel se hará privado.

## Números ocupados (para no repetir el choque de la 4.33)

| Qué | Último | Siguiente libre |
|---|---|---|
| Firmware en `main` de Orvy | 4.34 (imagen de la Flash, sin publicar) | 4.35 |
| Firmware publicado | 4.33 (`fw-v4.33`) | — |
| App publicada | 1.1.22 | 1.1.23 |

Resérvalos en la issue «Reservas» de Orvy antes de usarlos.
