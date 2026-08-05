# Complementos de OrbyGUI

OrbyGUI sabe manejar el teclado: perfiles, atajos, secuencias, iconos. Lo que
no sabe es qué hay al otro lado del PC. Un **complemento** es lo que traduce
«esta tecla» a «esto de fuera»: una lámpara de la red, un programa que expone
una API, un servidor de música, una impresora 3D, lo que sea que el ordenador
pueda alcanzar.

La instalación por defecto no trae ninguno. Se instalan desde
**Ajustes → Complementos**, y viven en la carpeta de datos del usuario
(`%APPDATA%\OrbyGUI\plugins\<id>\`), no dentro de la app: actualizar OrbyGUI no
se los lleva por delante.

> **Un complemento es un programa.** Corre dentro del proceso principal de
> OrbyGUI, en Node, sin caja de arena: puede leer y escribir archivos, salir a
> internet y lanzar otros programas. Instala solo los que vengan de un origen
> que conozcas. Por eso instalar uno pide confirmación explícita.

---

## Qué es un complemento

Una carpeta con dos archivos:

```
mi-complemento/
├── plugin.json     manifiesto
└── main.js         código (Node, CommonJS)
```

Para repartirlo, un `.zip` de ese contenido. `npm run pack:plugins` empaqueta
los que haya en `OrbyGUI/plugins/` y los deja en `release/plugins/`.

---

## El manifiesto (`plugin.json`)

```json
{
  "id": "mi-complemento",
  "name": "Mi complemento",
  "version": "1.0.0",
  "description": "Una frase de qué hace. Sale en la lista y bajo sus botones.",
  "author": "Quien lo firma",
  "icon": "plug",
  "apiVersion": 1,
  "main": "main.js"
}
```

| Campo | Obligatorio | Qué es |
|---|---|---|
| `id` | sí | Minúsculas, dígitos y guiones. Es el nombre de la carpeta y la clave de sus ajustes, así que **no se puede cambiar** sin perder lo que el usuario tenga asignado. |
| `name` | sí | Lo que se lee en la interfaz. |
| `apiVersion` | sí | Hoy `1`. Si no coincide con la que habla OrbyGUI, el complemento se rechaza al cargar en vez de reventar a mitad de una pulsación. |
| `version` | no | Se enseña junto al nombre. |
| `description`, `author` | no | Salen en la lista y en el aviso de instalación. |
| `icon` | no | Nombre de un icono de `src/icons.js` (`plug`, `sun`, `moon`, `bolt`, `wheel`, `lock`, `console`…). Por defecto `plug`. |
| `main` | no | Archivo de entrada. Por defecto `main.js`. |
| `legacy` | no | Solo para lo que antes venía dentro de la app; ver el final. |

---

## El módulo (`main.js`)

Exporta una **fábrica** que recibe la API del anfitrión y devuelve lo que el
complemento ofrece:

```js
module.exports = (api) => ({
  actions: [ /* … */ ],
  settings: { /* … */ },
  async run(step, settings) { /* … */ },
  async test(settings) { /* … */ },   // opcional
  dispose() { /* … */ },              // opcional
});
```

### `actions` — lo que se puede asignar a una tecla o a un mando

```js
actions: [
  { op: 'toggle', label: 'Encender / apagar', targets: ['key', 'click'] },
  { op: 'volumen', label: 'Volumen', targets: ['turn'], step: 5 },
]
```

| Campo | Qué es |
|---|---|
| `op` | Identificador de la acción. Es lo que se guarda en el perfil del usuario: cambiarlo rompe lo que ya tuviera asignado. |
| `label` | Lo que se lee en el botón. |
| `targets` | Dónde encaja: `key` (una tecla, pestaña *Multimedia*), `click` (la pulsación de un encoder), `turn` (el giro de un encoder o de la rueda). |
| `step` | Solo para `turn`: cuánto se mueve el valor **por muesca**. |

`key` y `click` son para órdenes sueltas («enciende»); `turn` es para lo que
tiene dirección («sube cinco»). El signo lo pone el propio hueco —horario sube,
antihorario baja—, así que el usuario elige lo mismo en los dos sentidos del
encoder y ya queda montado. El botón *Invertir giro* del inspector le da la
vuelta al par entero si el encoder está montado del revés.

### `settings` — lo que el complemento necesita saber

Se declaran los campos y OrbyGUI pinta la tarjeta él solo, en Ajustes:

```js
settings: {
  description: 'Texto al pie de la tarjeta.',
  fields: [
    { key: 'host', label: 'Dirección', type: 'text',
      placeholder: '192.168.1.50:8080', default: '192.168.1.50:8080' },
    { key: 'confirmar', label: 'Pedir confirmación', type: 'toggle', default: false },
    { key: 'modo', label: 'Modo', type: 'select',
      options: [{ value: 'a', label: 'Rápido' }, { value: 'b', label: 'Preciso' }] },
  ],
}
```

Tipos: `text`, `password`, `number`, `toggle`, `select`. Todos admiten
`default` y `hint` (una línea de ayuda debajo).

Los valores los guarda OrbyGUI en su configuración local, no el complemento, y
**sobreviven a desinstalarlo**: reinstalarlo recupera lo que costó ajustar.

### `run(step, settings)` — ejecutar

```js
async run(step, settings) {
  // step: { op: 'volumen', value: 5 }   (value es 0 en las acciones sin giro)
  // settings: los valores actuales de los campos declarados arriba
}
```

Lo llama el proceso principal cuando el teclado dispara la acción. Puede ser
`async`; si lanza, el error se registra en la consola y el resto de la macro
sigue.

> **Un giro son muchas llamadas.** El firmware manda un aviso **por cada
> muesca**, así que un giro rápido son veinte llamadas en un suspiro. Si lo que
> hay al otro lado no aguanta ese ritmo (una ESP32, una API con límite), agrupa
> los incrementos en el propio complemento antes de mandarlos: mira cómo lo
> hace `plugins/lampdesk/main.js`.

### `test(settings)` — el botón «Probar»

```js
async test(settings) {
  // devuelve { ok: true, detail: 'Responde: encendida, brillo 40 %' }
  //        o { ok: false, error: 'motivo' }
}
```

Si existe, la tarjeta de ajustes enseña un botón *Probar* que lo llama con lo
que haya escrito en los campos **en ese momento** (no con lo guardado): es lo
que uno intenta hacer al cambiar una dirección que no responde. Debe ser una
operación de solo lectura.

### `dispose()` — al desinstalar

Para soltar lo que quede corriendo (temporizadores, conexiones). Node no sabe
descargar un módulo ya cargado, así que sin esto seguiría vivo hasta cerrar la
app.

---

## La API del anfitrión (`api`)

| Miembro | Qué hace |
|---|---|
| `api.id` | El id del complemento. |
| `api.dir` | Su carpeta instalada, para leer archivos propios. |
| `api.settings()` | Los ajustes actuales (con los `default` aplicados). |
| `api.setSettings(patch)` | Guarda ajustes desde el propio complemento. |
| `api.log(...)` / `api.error(...)` | Consola, con el id delante. |

Todo lo demás es Node normal: `fetch`, `fs`, `child_process`, y cualquier
dependencia que el complemento traiga ya resuelta dentro de su carpeta (no se
ejecuta `npm install` al instalarlo).

---

## Cómo se guarda en el perfil del usuario

Una acción de complemento es una macro de un solo paso:

```json
{ "type": "plugin", "plugin": "mi-complemento", "op": "volumen", "value": 5 }
```

La tecla o el mando guardan en el teclado solo el **id de la macro**; el paso
vive en el PC (`%APPDATA%\OrbyGUI\orby-config.json`). Por eso estas acciones
necesitan OrbyGUI abierto —basta con el icono de la bandeja— y **no** hacen
falta «Guardar en Flash».

Si el complemento se desinstala o se desactiva, la tecla deja de hacer nada
pero conserva su ajuste, y el inspector lo dice: volver a instalarlo la
resucita.

---

## Probarlo mientras se escribe

1. Deja la carpeta donde te venga bien (por ejemplo `OrbyGUI/plugins/lo-mio/`).
2. **Ajustes → Complementos → Instalar complemento → Carpeta**.
3. Cada cambio en `main.js` necesita reinstalar (se vuelve a copiar) y reiniciar
   OrbyGUI: el módulo ya está cargado en el proceso principal.

Los `console.log` del complemento salen por la consola del proceso principal
(la terminal desde la que arranques `npm run dev`), no por la de la ventana.

---

## `legacy`: migrar lo que antes venía dentro de la app

La lámpara vivía dentro de OrbyGUI antes de que existieran los complementos.
Para que las teclas ya asignadas y los ajustes guardados sigan funcionando al
instalar el complemento, el manifiesto puede declarar qué dejó atrás:

```json
"legacy": { "stepType": "lamp", "configKey": "lamp" }
```

- `stepType`: los pasos guardados con ese `type` se reescriben a
  `{ type: 'plugin', plugin: '<id>', … }` conservando el resto de campos.
- `configKey`: esa clave de la configuración pasa a ser los ajustes del
  complemento (solo si no tenía ya los suyos) y desaparece del archivo.

Corre una sola vez, al cargar el complemento. Un complemento nuevo no necesita
esto para nada.
