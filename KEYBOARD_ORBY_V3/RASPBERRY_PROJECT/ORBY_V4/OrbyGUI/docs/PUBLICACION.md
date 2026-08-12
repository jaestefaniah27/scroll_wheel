# Publicar ORBY como producto

Qué hace falta para que ORBY deje de parecer un proyecto de escritorio: que
Windows no avise al instalar la app, que el teclado se presente como un aparato
con identidad propia, y que haya una página donde descargarlo.

Este documento es la parte operativa del punto correspondiente de
[TODO.md](../../TODO.md). Lo que ya está hecho está hecho; lo que queda son
gestiones con terceros o cosas que cuestan dinero.

---

## 1. Identidad USB

### Estado

Hecho en `src/usb_descriptors.c`:

- VID y PID salen de `ORBY_USB_VID` / `ORBY_USB_PID`, en un solo sitio.
- `bcdDevice` vale `0x0410`, la versión de firmware real (4.1). Antes decía 1.0.
- El número de serie se genera con el ID único de la flash de la Pico, así que
  cada teclado tiene el suyo. Antes todos decían `123456`.
- Los strings dicen lo que el aparato es: `Orby` / `Orby V4` / `Orby V4 Control`
  / `Orby V4 Input`.

Pendiente: **el VID sigue siendo `0xCafe`**, que es el de los ejemplos de
TinyUSB. No es nuestro y no se puede distribuir con él.

### Pedir un PID a Raspberry Pi

Raspberry Pi reparte PIDs gratis bajo su VID `0x2E8A` a productos basados en
RP2040. Es la vía buena: no cuesta nada y el aparato queda enumerado bajo un VID
legítimo con un PID que es solo nuestro.

> Un VID propio de USB-IF cuesta unos 6.000 $ de una vez. No tiene sentido aquí.
> La otra alternativa gratis es pid.codes (VID `0x1209`), que exige proyecto de
> código abierto.

El proceso es una *pull request* al repositorio `raspberrypi/usb-pid`. **Lee su
README antes de abrirla**, porque el formato de la tabla ha cambiado alguna vez.
El contenido a aportar es este:

- **Producto:** ORBY V4
- **Descripción:** teclado macro de 12 teclas con 10 pantallas OLED, dos
  encoders y rueda de scroll de alta resolución, sobre RP2040.
- **Interfaces USB:** HID (teclado, ratón con scroll de alta resolución,
  consumidor) + CDC para la configuración.
- **Repositorio:** https://github.com/jaestefaniah27/scroll_wheel
- **Contacto:** jaestefaniah27@gmail.com

### Cuando lo concedan

Cambiar **en el mismo commit** los dos sitios, o la app deja de encontrar el
teclado:

1. `src/usb_descriptors.c` → `ORBY_USB_VID` a `0x2E8Au` y `ORBY_USB_PID` al
   asignado.
2. `OrbyGUI/src-tauri/src/serial.rs` → aceptar el VID nuevo junto a
   `VID_TECLADO`, **sin quitar** el `0xCAFE`: los teclados ya flasheados siguen
   ahí fuera.

Ojo con una consecuencia: Windows cachea el *report descriptor* HID por VID/PID.
Al cambiar el VID, el host trata el teclado como un aparato nuevo y vuelve a
leer el descriptor, que es lo que queremos. Pero también significa que las
asociaciones que Windows tuviera guardadas (número de puerto COM, por ejemplo)
se pierden: el teclado aparecerá en un COM distinto la primera vez.

---

## 2. Firmar la aplicación

El aviso azul al instalar es **SmartScreen**, y sale porque el `.exe` no va
firmado. No hay forma de quitarlo sin una firma: la reputación de SmartScreen
para binarios sin firmar se calcula por hash de fichero, así que cada versión
nueva empieza de cero y con el volumen de este proyecto no se limpia nunca.

Un certificado autofirmado **no sirve**: SmartScreen lo ignora y encima obliga
al usuario a instalar la CA a mano. Es peor que no firmar.

Desde junio de 2023 el CA/B Forum obliga a guardar la clave privada en hardware
(FIPS 140-2 nivel 2). Ya no existe la opción de un `.pfx` en el disco.

### Opciones, de menos a más dinero

| Vía | Coste | Requisitos | Resultado |
|---|---|---|---|
| **SignPath Foundation** | 0 € | proyecto de código abierto con licencia OSI, repo público, build en CI | certificado OV real |
| **Microsoft Store** | ~19 €, pago único | cuenta de desarrollador individual, empaquetar en MSIX | firma de Microsoft, **cero SmartScreen** |
| Azure Trusted Signing | ~10 €/mes | entidad legal verificada | OV; la reputación se acumula |
| OV con token USB | 250–400 €/año | entidad legal verificada | igual que el anterior |
| EV con token USB | 400–700 €/año | entidad legal registrada | el mejor, casi inmediato |

**Empezar por SignPath.** Es gratis y es la única opción que no exige ser una
empresa. Si dicen que no, la Store por 19 € es lo siguiente.

### Solicitud a SignPath Foundation

Se pide en `signpath.org` (sección Foundation / open source). Borrador:

> Asunto: Free code signing for ORBY V4 — open-source macro keyboard
>
> Hello,
>
> I maintain ORBY V4, an open-source macro keyboard built on the RP2040. The
> project has two parts: the firmware for the keyboard itself (C++, Pico SDK)
> and OrbyGUI, a Tauri desktop app that configures it.
>
> I would like to apply for a free code signing certificate for the OrbyGUI
> Windows installer. Right now it ships unsigned, and every user has to click
> through the SmartScreen warning, which is exactly the habit I do not want to
> teach people who install my software.
>
> - Repository: https://github.com/jaestefaniah27/scroll_wheel
> - What gets signed: the NSIS installer and the app executable, both produced
>   by the Tauri bundler.
> - Build: from public sources, reproducible from a tagged commit.
>
> One question before I apply formally: I am considering selling optional
> profile packs for the app in the future, while the app and firmware stay
> open source. Would that conflict with the Foundation's eligibility criteria?
>
> Thank you,
> Jorge

**Antes de mandarlo hay que decidir la licencia.** El repositorio no tiene
ninguna, y SignPath exige una licencia OSI. Es una decisión con consecuencias:
una licencia permisiva (MIT, Apache-2.0) deja que cualquiera fabrique y venda
el teclado; una copyleft fuerte (GPL-3.0) obliga a quien lo haga a publicar sus
cambios. Para hardware que se quiere vender, GPL-3.0 en el firmware y MIT en la
app es un reparto habitual. **No está decidido: hay que elegir a mano.**

### Cuando haya certificado

Enganchar la herramienta de firma al empaquetador, en `src-tauri/tauri.conf.json`:

```jsonc
"bundle": {
  "windows": {
    "signCommand": "signtool sign /fd sha256 /tr http://timestamp.digicert.com %1"
  }
}
```

Hacerlo el mismo día que se contrate: el trabajo es gratis pero no sirve de nada
suelto.

**Ojo: la firma Authenticode y la firma del actualizador son cosas distintas** y
no se sustituyen. La minisign de la sección 3 protege la *descarga* —que el
`.exe` que se instala solo es el que publicamos—; la Authenticode protege la
*reputación* ante SmartScreen y el antivirus. Hoy está la primera y falta la
segunda, y eso es justo lo que hace que la actualización silenciosa tenga un
riesgo: se instala sola, sin nadie mirando la pantalla, sobre un binario que
Windows no reconoce.

---

## 3. Publicar una versión

**La app se actualiza sola y en silencio**, así que publicar una versión es
también actualizar a todo el mundo sin que nadie apriete nada. De ahí que este
apartado tenga más ceremonia de la que parece merecer.

### Las claves de firma (una sola vez)

```powershell
cd OrbyGUI
npm run tauri signer generate -- -w "$env:USERPROFILE\.tauri\orby.key"
```

Salen dos: la **pública**, que va escrita en `src-tauri/tauri.conf.json`
(`plugins.updater.pubkey`), y la **privada** con su contraseña, que **no entra
nunca en el repositorio**. Sin la privada no se puede firmar una actualización;
si se pierde, hay que publicar una clave nueva y todas las copias instaladas
dejan de poder actualizarse solas hasta que alguien las reinstale a mano.

### Cada versión

Primero el número de versión, en `tauri.conf.json`, `Cargo.toml` y
`package.json`, que tienen que decir lo mismo. El panel Orby Manager los toca de
una (`tools/orby-manager`, bump de la app) y avisa si se han separado.

```powershell
cd OrbyGUI
$env:TAURI_SIGNING_PRIVATE_KEY = Get-Content "$env:USERPROFILE\.tauri\orby.key" -Raw
$env:TAURI_SIGNING_PRIVATE_KEY_PASSWORD = "<la contraseña>"
npm run tauri:build
```

Salen tres ficheros en `src-tauri/target/release/bundle/nsis/`:

| Fichero | Para qué |
|---|---|
| `OrbyGUI_X.Y.Z_x64-setup.exe` | El instalador |
| `OrbyGUI_X.Y.Z_x64-setup.exe.sig` | Su firma minisign |
| `latest.json` | Lo que consulta el actualizador: versión, notas, URL y firma |

**Los tres van a la release, o la release no sirve.** El actualizador pide el
`latest.json`, ese apunta al `.exe` y comprueba su firma contra el `.sig`.

> **Si el build corre sin `TAURI_SIGNING_PRIVATE_KEY`, no avisa: compila igual,
> pero sin firma y sin `latest.json`.** Sale un `.exe` perfecto y una release
> que ningún actualizador va a ver, y nadie se entera hasta que alguien pregunta
> por qué su OrbyGUI sigue en la versión de hace tres meses. El pipeline del
> panel Orby Manager comprueba la variable **antes** de compilar justo por eso.

Y la etiqueta, `vX.Y.Z`, **sin `--prerelease`**: la app pregunta por
`releases/latest`, y una release marcada como prerelease no sale por ahí. Es la
regla contraria a la del firmware, que sí va siempre como prerelease (ver
[COMPATIBILIDAD.md](../../docs/COMPATIBILIDAD.md)).

### La huella, en las notas

Mientras no haya firma Authenticode, la huella es lo único que distingue nuestro
instalador de cualquier otro fichero con el mismo nombre:

```powershell
Get-FileHash .\src-tauri\target\release\bundle\nsis\OrbyGUI_*-setup.exe -Algorithm SHA256
```

En las notas de la versión, pegar la salida y el recordatorio del aviso:

```markdown
## Instalación

Windows mostrará el aviso de SmartScreen: el instalador todavía no va firmado
(ver docs/PUBLICACION.md). Para continuar: **Más información → Ejecutar de
todas formas**.

Antes de ejecutarlo, comprueba que el fichero es el que publicamos:

    Get-FileHash .\OrbyGUI-Setup.exe -Algorithm SHA256

    SHA256: <pegar aquí>
```

---

## 4. La landing

Vive en `web/index.html`, en la raíz del proyecto ORBY_V4. Es un único fichero
estático, sin build ni dependencias.

Para que se publique:

1. En GitHub → Settings → Pages → **Source: GitHub Actions**.
2. Empujar a `main`. El workflow `.github/workflows/pages.yml` sube la carpeta
   `web/` tal cual.

Queda en `https://jaestefaniah27.github.io/scroll_wheel/`.

Un dominio propio cuesta unos 12 €/año y se conecta desde la misma pantalla de
Pages (campo *Custom domain* + un registro CNAME en el proveedor del dominio).

---

## 5. Lo que queda bloqueado por dinero

Nada de esto se puede adelantar sin presupuesto, pero conviene saber el orden y
el motivo:

- **Certificado de firma** — ver la tabla de arriba. Sin él, SmartScreen sigue.
- **Marca "Orby"** — la CA verifica el nombre legal al emitir el certificado.
  Además, comprobar antes que la marca esté libre.
- **Marcado CE** para vender el teclado en la UE: directiva EMC 2014/30/UE más
  RoHS. La declaración la firma uno mismo, pero hace falta informe de
  laboratorio (EN 55032 emisiones clase B, EN 55035 inmunidad): 1.500–4.000 €.
  Es el mayor coste del proyecto. Solo cuando el hardware esté congelado.
- **Registro RAEE** en España y Ecoembes por el embalaje, que van con lo
  anterior.
- **FCC Part 15 clase B** si se vende en EEUU.
- **Kickstarter**: exige entidad legal, cuenta bancaria, vídeo y coste unitario
  cerrado. Y enviar a la UE sin CE no es una opción.

---

## Apéndice: qué se descartó

**Descriptores Microsoft OS 2.0 (BOS/WCID).** Estaban en la lista y se han
dejado fuera a propósito. En un dispositivo compuesto CDC + HID, Windows ya
enlaza `usbser.sys` por clase desde Windows 10: los descriptores MS OS solo
servirían para colgar una propiedad de registro (`DeviceInterfaceGUIDs`) que
hoy no usa nadie, porque `serial.js` encuentra el puerto por VID/PID. A cambio
obligan a subir `bcdUSB` a `0x0201` y a añadir un descriptor binario escrito a
mano; si sale mal, el aparato no enumera. Coste real, beneficio cero.

Si algún día `serial.js` necesita localizar el puerto por GUID en vez de por
VID/PID, se retoma. Y si se retoma: **nunca declarar un ID de compatibilidad**
en el subconjunto de función del CDC. Declarar `WINUSB` ahí sustituye a
`usbser.sys` y deja el puerto serie inservible.
