# Plan: dejar la actualización automática funcionando de verdad

> **Para quien lo ejecute.** Este plan está escrito para poder seguirse **al pie de la
> letra**, sin decidir nada. Cada paso lleva el comando exacto, lo que tiene que salir y
> qué hacer si sale otra cosa. Si algo no encaja con lo que dice aquí, **para y dilo**: no
> improvises una alternativa ni «arregles» el código para que cuadre. Es mucho más barato
> corregir este documento.
>
> Todo esto se hace **en Windows, en el PC del autor**, con el teclado enchufado. Nada de
> esto se puede hacer desde un contenedor Linux, y por eso quedó pendiente.

## Qué falta y por qué

El código de la actualización automática **está entero y probado**. Lo que falta es lo que
solo puede hacer una persona en su máquina:

1. **Generar el par de claves de firma.** El actualizador descarga el instalador y
   comprueba su firma antes de ejecutarlo. Sin claves no hay firma, y sin firma no hay
   actualización. La privada no puede vivir en el repositorio, así que esto no se pudo
   dejar hecho.
2. **Comprobarlo sobre el teclado.** Que la app se actualice sola y que el teclado se
   flashee estando ocioso son las dos cosas que este trabajo prometía, y ninguna se ha
   visto funcionar todavía.
3. **Publicar una release con los tres ficheros.** Hasta que exista una, no hay nada que
   descargar.

Estado actual, comprobado: `npm test` 22/22 en `OrbyGUI/` y 6/6 en `tools/orby-manager/`,
`cargo test -p orby-core` 137/137, `cargo check --target x86_64-pc-windows-gnu` limpio,
`verifica_plan_tauri.sh` con 0 discrepancias y **un PENDIENTE**, que es justo el punto 1.

## El orden importa, y no es el obvio

**Publicar exige estar en `main`.** El pipeline del panel se niega en el primer paso:
`Se publica desde main, y la rama actual es «…»`. Y `main` **no tiene la migración a
Tauri**: todo vive en `claude/task-13-auto-updates-e09ces`.

Eso obliga a este orden, y no a otro:

```
1. Claves          ─┐
2. Build de prueba  ├─ en la rama, sin tocar main
3. Probar en el teclado lo que no necesita release  ─┘
4. Fusionar a main   ← decisión del autor
5. Publicar          ← solo se puede desde main
6. Probar la actualización automática de la app  ← necesita la release
```

**No intentes publicar antes del paso 4.** Y no fusiones antes del paso 3: fusionar es
mucho más fácil que deshacerlo.

## Antes de empezar: comprobar dónde estás

```powershell
cd <repo>\KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4\OrbyGUI
git branch --show-current
```

**Tiene que decir `claude/task-13-auto-updates-e09ces`.** Si dice `main`, para: la
migración a Tauri no está allí y nada de esto existe. Cámbiate con
`git checkout claude/task-13-auto-updates-e09ces` y vuelve a comprobarlo.

```powershell
git status --porcelain
```

Tiene que salir **vacío**. Varios pasos de abajo se niegan a correr con el árbol sucio.

---

# Paso 1 — Generar el par de claves (una sola vez en la vida)

## 1.1 Generar

```powershell
cd <repo>\KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4\OrbyGUI
npx tauri signer generate -w "$env:USERPROFILE\.tauri\orby.key"
```

Pedirá una contraseña por teclado (dos veces). **Ponle una y apúntala donde guardes las
contraseñas.** Se puede dejar vacía dando a Enter, pero entonces cualquiera que copie el
fichero puede firmar actualizaciones que tu app instalará sola y sin preguntar.

Salida esperada, literalmente:

```
Your keypair was generated successfully:
Private: C:\Users\<tu-usuario>\.tauri\orby.key (Keep it secret!)
Public: C:\Users\<tu-usuario>\.tauri\orby.key.pub
```

**Si el fichero ya existe**, el comando falla en vez de sobrescribir. Eso es lo correcto:
significa que ya tienes claves. **No uses `--force`** — perder la privada deja a todas las
copias instaladas sin poder actualizarse nunca más. Salta al 1.2 con las que ya hay.

## 1.2 Pegar la clave pública en la configuración

```powershell
Get-Content "$env:USERPROFILE\.tauri\orby.key.pub" -Raw
```

Sale **una sola línea larga en base64**, de unos 150 caracteres, que empieza por
`dW50cnVzdGVk`. Cópiala entera, sin saltos de línea.

En `OrbyGUI\src-tauri\tauri.conf.json`, sustituye el marcador:

```jsonc
"pubkey": "PEGAR-AQUI-LA-CLAVE-PUBLICA-DE-npm-run-tauri-signer-generate"
```

por lo que acabas de copiar:

```jsonc
"pubkey": "dW50cnVzdGVkIGNvbW1lbnQ6...."
```

**No toques nada más de ese fichero.** `endpoints` ya apunta donde debe y
`installMode: "passive"` es lo que hace que el instalador no plante diálogos.

## 1.3 Comprobar

```powershell
cd <repo>\KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4
bash tools/test/verifica_plan_tauri.sh
```

- **Antes** decía: `PENDIENTE  falta generar el par de claves…`
- **Ahora**: `ok   la clave publica del actualizador esta puesta`
- Y al final, `TODO CUADRA: 0 discrepancias`.

Si sigue saliendo PENDIENTE, el marcador no se sustituyó bien: vuelve al 1.2.

## 1.4 Commit

```powershell
cd <repo>
git add KEYBOARD_ORBY_V3/RASPBERRY_PROJECT/ORBY_V4/OrbyGUI/src-tauri/tauri.conf.json
git commit -m "feat(tauri): clave publica del actualizador"
git push -u origin claude/task-13-auto-updates-e09ces
```

> **La clave privada NO se toca aquí.** Vive en `%USERPROFILE%\.tauri\` y ahí se queda. Si
> en algún momento aparece un fichero `.key` (sin `.pub`) dentro del repositorio,
> **bórralo y no lo subas**.

---

# Paso 2 — Comprobar que el build produce los tres ficheros

Esto se hace **antes** de publicar nada, porque es donde se ve si la firma funciona.

## 2.1 Cerrar OrbyGUI

`tauri build` **falla si hay un OrbyGUI corriendo**, y el error (`os error 32`) no dice
cuál es el problema. Ciérrala desde el icono de la bandeja → **Salir**. Comprobar:

```powershell
Get-Process orby-app,OrbyGUI -ErrorAction SilentlyContinue
```

Tiene que **no devolver nada**. (Dos nombres porque son dos binarios: `orby-app.exe` es el
que deja el build en `target\release\`, y `OrbyGUI.exe` es el que instala el `.exe`. El que
bloquea el build es el primero, pero conviene cerrar los dos.)

## 2.2 Compilar con la clave

```powershell
cd <repo>\KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4\OrbyGUI
$env:TAURI_SIGNING_PRIVATE_KEY = Get-Content "$env:USERPROFILE\.tauri\orby.key" -Raw
$env:TAURI_SIGNING_PRIVATE_KEY_PASSWORD = "<la contraseña del paso 1.1>"
npm run tauri:build
```

Tarda varios minutos la primera vez.

> **Ojo con las tuberías.** Si encadenas la salida (`| tee`, `| Select-String`), el código
> de salida pasa a ser el del último eslabón y **un build fallido se lee como exitoso**.
> Deja que escriba en la consola.

## 2.3 Comprobar los tres ficheros

```powershell
dir src-tauri\target\release\bundle\nsis\
```

Tienen que estar **los tres**:

| Fichero | Qué es |
|---|---|
| `OrbyGUI_1.0.0-alpha_x64-setup.exe` | El instalador |
| `OrbyGUI_1.0.0-alpha_x64-setup.exe.sig` | Su firma |
| `latest.json` | Lo que consulta el actualizador |

**Si falta el `.sig` o el `latest.json`, el build ha corrido sin ver la clave.** Es el
fallo más caro de esta lista porque **no da ningún error**: sale un `.exe` perfecto y una
release que ningún actualizador verá jamás. Vuelve al 2.2 y comprueba que las variables
están puestas *en la misma consola* desde la que lanzas el build:

```powershell
$env:TAURI_SIGNING_PRIVATE_KEY.Length   # tiene que ser > 0
```

Mira además el `latest.json`:

```powershell
Get-Content src-tauri\target\release\bundle\nsis\latest.json
```

Tiene que llevar `"version"`, una `"signature"` larga y una `"url"` de GitHub apuntando a
una release que aún no existe. Esa URL se predice a partir de la etiqueta; por eso el
fichero se genera antes de publicar.

---

# Paso 3 — Probar sobre el teclado lo que no necesita release

Instala el `.exe` que acabas de compilar (doble clic, sobre la instalación existente).
Todo lo de este paso funciona **sin haber publicado nada**.

## 3.1 El firmware se actualiza solo

Es la comprobación más importante de todo el plan, porque es la única que puede romper
algo físico.

Condiciones que se tienen que dar **todas** (si falta una, no pasa nada, y eso es correcto):

- Teclado conectado y reconocido.
- Hay una versión de firmware más nueva publicada, y no pasa de `FW_RECOMMENDED`.
- **Cinco minutos sin tocar el teclado**: ni teclas ni giros del mando.
- Sin cambios pendientes de guardar en Flash.
- Sin grabación ni secuencia en marcha.

**Prueba positiva:**

1. Deja el teclado con una versión de firmware por debajo de la publicada.
2. Abre OrbyGUI, guarda cualquier cambio pendiente y **no toques el teclado**.
3. Espera. El sondeo es cada minuto, así que entre 5 y 6 minutos.

Tiene que reiniciarse solo en modo cargador, copiarse el `.uf2`, volver, y salir un aviso
`Firmware actualizado a X`.

**Prueba negativa, que importa más:** repite lo anterior pero **teclea algo cada minuto**.
**No se puede flashear nunca.** Si se flashea mientras estás usando el teclado, para todo
y dilo: es el peor fallo posible de esta función.

**Segunda prueba negativa:** haz un cambio, **no** lo guardes en Flash, y deja el teclado
quieto cinco minutos. Tampoco se puede flashear — el reinicio en el cargador se llevaría
por delante lo que solo está en RAM.

Si no pasa nada y no sabes por qué, arranca con `npm run tauri:dev`: la consola dice el
motivo exacto por el que no se ha disparado.

## 3.2 El interruptor del firmware

Ajustes → Firmware del teclado → **«Actualizar el teclado automáticamente»**.

- **Apagado**: vuelve el aviso de siempre al conectar («Hay firmware nuevo… Ajustes →
  Firmware del teclado») y hay que darle al botón.
- **Encendido**: ese aviso **no** sale, porque sería mentira: para cuando llegaras a
  Ajustes ya estaría hecho.
- **El botón «Actualizar teclado» sigue estando en los dos casos**, a propósito:
  reinstalar el mismo firmware es como se recupera un teclado que se quedó a medias, y eso
  el automático no lo hará nunca porque para él ya está al día.

## 3.3 El interruptor de la app

Ajustes → Aplicación → **«Instalar las actualizaciones automáticamente»**.

- Al apagarlo, el texto de debajo tiene que cambiar y decir que no se instala nada sin que
  lo pidas.
- **Tiene que sobrevivir al reinicio**: apágalo, cierra la app del todo (bandeja → Salir),
  vuelve a abrirla y mira que sigue apagado. Vive en la configuración local justo para eso.
- Vuelve a encenderlo antes de seguir.

## 3.4 Autoarranque escondido

Es la comprobación que quedó pendiente de la Tarea 12:

1. Ajustes → Autoarranque → activar.
2. Cerrar sesión de Windows y volver a entrar.
3. La app tiene que arrancar **escondida en la bandeja**, sin ventana, con el teclado
   detectado.
4. Abrir Ajustes: el interruptor tiene que seguir encendido.

```powershell
Get-ItemProperty 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Run' -Name OrbyGUI
# Esperado: "...\OrbyGUI.exe" --hidden
```

---

# Paso 4 — Fusionar a `main`

> **DECISIÓN DEL AUTOR. No lo hagas sin que lo diga explícitamente.** Fusionar es mucho
> más fácil que deshacerlo, y hasta aquí todo era reversible.

`main` está en `b9bb08c` y **no tiene la migración a Tauri**. Solo cuando el paso 3 esté
comprobado y el autor dé el visto bueno:

```powershell
cd <repo>
git checkout main
git merge --no-ff claude/task-13-auto-updates-e09ces
git push origin main
```

Comprobar que ha llegado entero:

```powershell
git log --oneline -1
dir KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4\OrbyGUI\src-tauri
```

La carpeta `src-tauri` tiene que existir en `main`, y **no** tiene que existir
`OrbyGUI\electron`.

---

# Paso 5 — Publicar la primera versión

Desde `main`, y con el árbol limpio.

## 5.1 Subir el número de versión

Hoy la app está en `1.0.0-alpha`. Hay **tres ficheros** que tienen que decir lo mismo:
`src-tauri/tauri.conf.json`, `src-tauri/Cargo.toml` y `package.json`.

**No los edites a mano.** Usa el panel, que los toca a la vez o a ninguno:

```powershell
cd <repo>\KEYBOARD_ORBY_V3\RASPBERRY_PROJECT\ORBY_V4\tools\orby-manager
npm install
node server.js
```

Abre `http://localhost:7788` → sección **Publicar** → **Bump — App** → tipo `patch` → Bump.

`1.0.0-alpha` con `patch` da **`1.0.0`**: a una preversión no se le suma, se gradúa. Es lo
que semver dice que viene después.

Comprobar en la sección Versiones que la tarjeta **App en el repo** enseña **un solo
número**. Si enseña tres separados por `/` y está en rojo, se han desincronizado: iguálalos
a mano y repite.

```powershell
cd <repo>
git add -A
git commit -m "chore: version 1.0.0"
git push origin main
```

## 5.2 Publicar

Antes de pulsar nada, en la consola desde la que vas a arrancar el panel:

```powershell
$env:GH_TOKEN = "<tu token de GitHub>"
$env:TAURI_SIGNING_PRIVATE_KEY = Get-Content "$env:USERPROFILE\.tauri\orby.key" -Raw
$env:TAURI_SIGNING_PRIVATE_KEY_PASSWORD = "<la contraseña>"
node server.js
```

Panel → **Publicar** → **Release — App**. El pipeline comprueba las dos variables **antes**
de compilar, precisamente para no gastar un build entero y descubrirlo al final.

Los seis pasos y qué esperar:

| Paso | Qué hace | Si falla |
|---|---|---|
| 1. Comprobaciones | Token, clave, árbol limpio, rama `main`, que la release no exista ya | El mensaje dice cuál de las cinco |
| 2. Compilar | `npm run tauri:build` | Casi siempre es un OrbyGUI abierto |
| 3. Reunir ficheros | Busca el `.exe`, el `.sig` y el `latest.json` | Falta el `.sig`: se compiló sin clave |
| 4. Crear la release | `prerelease: false`, a propósito | — |
| 5. Subir | Los tres ficheros | Reintenta; suele ser red |
| 6. Verificar | Relee la release y exige los tres | — |

**Alternativa a mano**, si el panel no arranca: crea la release en GitHub con etiqueta
`v1.0.0`, **sin marcar «This is a pre-release»**, y sube los tres ficheros. Esa casilla
importa: la app pregunta por `releases/latest`, y una prerelease no sale por ahí.

## 5.3 Comprobar la release desde fuera

```powershell
curl.exe -sL https://github.com/jaestefaniah27/scroll_wheel/releases/latest/download/latest.json
```

Tiene que devolver el JSON. **Si devuelve 404 o una página HTML**, o la release no está
publicada, o le falta el `latest.json`, o hay una release de firmware ocupando el `latest`
(ver la tabla del final).

---

# Paso 6 — Comprobar que la app se actualiza sola

Esto es lo que valida el trabajo entero, y necesita la release del paso 5.

1. Instala **una versión más baja** que la publicada. La forma más rápida: baja los tres
   ficheros de versión a `0.9.0`, `npm run tauri:build`, instala ese `.exe`, y devuelve los
   ficheros con `git checkout .`.
2. Abre la app y **no toques nada**. La primera consulta es a los ~5 segundos de arrancar.

**Lo que tiene que pasar solo:**

- La insignia de la barra superior dice `Descargando N%`, luego `Instalando 1.0.0`.
- La app se cierra y **vuelve a abrirse sola**.
- Ajustes → Aplicación → **Versión instalada** dice `1.0.0`.

**Repetir con la ventana escondida.** Cierra la ventana (se va a la bandeja), espera, y
comprueba que se ha actualizado igual. Ese camino es el que puede fallar por el interceptor
de cierre, y es el que más se usa: la app vive en la bandeja.

**Con el interruptor apagado**, en cambio, tiene que salir el botón **Instalar y
reiniciar** y no pasar nada hasta que lo pulses.

> **Si aparece SmartScreen o el antivirus y la actualización se queda parada**, eso es lo
> esperado hoy y está avisado: el instalador **no va firmado con certificado de código**.
> No es un fallo del actualizador. Lo barato que lo arreglaría está en `docs/TODO.md`: la
> cuenta de la Microsoft Store.

---

# Cuando algo no funciona

| Síntoma | Causa casi segura | Qué hacer |
|---|---|---|
| El build no deja `.sig` ni `latest.json` | `TAURI_SIGNING_PRIVATE_KEY` no estaba en esa consola | Paso 2.2. **No da error**: hay que mirar los ficheros |
| `os error 32` al compilar | Hay un `orby-app.exe` corriendo | Bandeja → Salir, o `taskkill /IM orby-app.exe /F` |
| «Se publica desde main, y la rama actual es…» | Estás en la rama de trabajo | Es correcto. Paso 4 antes que el 5 |
| La app dice «versión nueva sin latest.json firmado» | La release se publicó sin los tres ficheros | Sube el `.sig` y el `latest.json` a esa release |
| La app dice «La última release es de firmware» | Se publicó una `fw-v*` sin marcar prerelease | Márcala como prerelease en GitHub. El workflow ya lo hace solo desde este trabajo |
| El `latest.json` da 404 | Igual que la fila anterior, o la release está en borrador | Comprueba que está publicada, no en draft |
| La app no ve la versión nueva y los números parecen bien | Preversiones | `1.0.0-alpha` es **anterior** a `1.0.0`. Si la instalada es `1.0.0` y publicas `1.0.0-beta`, no la verá, y es correcto |
| El bump se niega: «no tiene forma x.y.z» | Los tres ficheros no dicen lo mismo | Iguálalos a mano y repite |
| SmartScreen para la actualización silenciosa | El instalador no va firmado con Authenticode | Es lo esperado hoy. Ver `docs/TODO.md` |
| El firmware no se flashea nunca | Alguna guarda | Normal si tocas el teclado. `npm run tauri:dev` dice el motivo exacto en consola |
| El firmware se flashea mientras trabajas | **Fallo grave** | Para, apaga el interruptor y dilo |

# Lo que NO hay que hacer

- **No uses `--force` al generar claves.** Perder la privada deja a todas las copias
  instaladas sin actualizaciones para siempre.
- **No subas la clave privada** al repositorio, ni al historial de git, ni a las notas de
  la release.
- **No fusiones a `main` sin que lo pida el autor**, y nunca antes del paso 3.
- **No publiques una release de la app como prerelease.** No saldría por `releases/latest`
  y ninguna copia la vería.
- **No publiques una release de firmware sin `--prerelease`.** Es el fallo contrario y
  rompe el actualizador de la app. El workflow ya lo hace bien; esto vale para las que se
  publiquen a mano.
- **No edites los tres ficheros de versión a mano** si puedes usar el panel.
- **No bajes el tiempo de ocio de los cinco minutos** para que las pruebas vayan más
  rápidas sin devolverlo después. Está en `src/firmware-decide.mjs` (`OCIO_MS`) y hay
  tests que lo usan como referencia.
