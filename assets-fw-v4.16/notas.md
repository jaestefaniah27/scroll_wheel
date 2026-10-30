# Firmware 4.16

## Un mando como menú: mantener, girar y soltar

En Ajustes → Tecla 10 → **Mando de menú** eliges el mando izquierdo o el derecho. Ese mando:

- **Girar**: cambia de página.
- **Mantenerlo pulsado**: abre el selector de perfiles. Sin soltar, giras hasta el perfil que quieras y **al soltar el teclado cambia a él**. Si sueltas sin girar, no cambia nada.
- **Mantenerlo con SUPER**: lo mismo con el gestor de páginas, que ahora tiene cursor.
- Un toque corto lo deja abierto hasta confirmar con otro clic, como en la 4.15.

Ese mando deja de configurarse por perfil (lo que tuviera asignado se conserva y vuelve si lo quitas). El mismo gesto vale para cualquier pulsación configurada como «Abrir menú» (gestor de páginas, menú principal) y para los dos mandos pulsados 2 s: girando antes de soltar se entra en la opción elegida.

**Con el menú abierto desde un mando, las teclas hacen lo que tengan asignado** en vez de ser atajos de selección.

## Invertir el sentido de mandos y rueda

En Ajustes → **Sentido de mandos y rueda**: mando izquierdo, derecho y rueda, por separado. Vale para todos los perfiles y también para los menús del teclado, y es independiente de la inversión de la rueda de cada perfil.

Al actualizar no cambia nada de lo que ya tienes: ningún mando invertido ni de menú hasta que lo elijas, y el formato de la Flash es el mismo.

## Instalación

Lo más cómodo: Orvy avisa al conectar el teclado y lo instala sola cuando no lo estás usando, sin tocar el botón BOOTSEL.

A mano: conecta el teclado con el botón **BOOTSEL** pulsado, aparece una unidad `RPI-RP2` y copias dentro `ORBY_V4-fw-4.16.uf2`.

    SHA256: 947620228dd2d160888c6d85ffa951cd97aefeab8c25d1bc39f3ac84b6c88137

## Compatibilidad con Orvy

| Firmware | Qué trajo | OrbyGUI |
|---|---|---|
| **4.16** | **Sentido de mandos y rueda, y el mando de menú.** `SET_ROTINV:<0..7>` (`ROTINV=1`) invierte el giro del encoder izquierdo (1), el derecho (2) y la rueda (4) para todo lo demás —acciones, menús y páginas—, sin tocar la telemetría `ENC:`/`WHEEL:`; es independiente de la inversión de la rueda por perfil. `SET_MENUENC:<0\|1\|2>` (`MENUENC=1`) deja un mando como menú: girarlo cambia de página y mantenerlo pulsado (250 ms) abre el selector de perfiles —con SUPER, el gestor de páginas—, se gira sin soltar hasta el elemento y **al soltar se elige**; un toque corto lo deja abierto hasta confirmar con otro clic. Lo que los perfiles tengan en ese mando se ignora. El mismo gesto sirve para cualquier clic con `ROT_MENU` (gestor de páginas, menú principal) y para los dos mandos pulsados 2 s (girando antes de soltar se entra en la opción). El gestor de páginas ya tiene cursor que mueven los mandos. **Con el menú abierto desde un mando, las teclas hacen lo que dice su página** en vez de ser atajos de selección. Ambos ajustes van en `GET_STATE` (`STATE:ROTINV`, `STATE:MENUENC`) y en bytes de relleno de los ajustes: no cambia el formato de la Flash | Orvy 1.1.3 o posterior para usar lo nuevo; con una anterior funciona como la 4.15 |
| **4.15** | **Los mandos llevan páginas y menús, y la tecla de menú se puede dejar libre** (`MENUKEY=1`). Dos tipos de acción de mando nuevos: `ROT_PAGE` (6, el giro pasa de página; como los desplazamientos, una acción cubre los dos sentidos) y `ROT_MENU` (7, el clic abre un menú: keycode 0 perfiles, 1 gestor de páginas, 2 menú principal). `SET_MENUKEY:<0\|1>` deja la tecla 10 como la de menú (0) o como una tecla normal (1); va en `GET_STATE` (`STATE:MENUKEY`) y se guarda en el byte de relleno de los ajustes, así que no cambia el formato de la Flash. Dentro de los menús navegan los dos mandos, no solo el izquierdo, y los dos mandos pulsados 2 s abren el menú principal desde cualquier menú | Orvy 1.1.3 o posterior para usar lo nuevo; con una anterior funciona como la 4.14 |
| **4.14** | Caben **255 secuencias** en el teclado en vez de 64 (`MAXMACROS=255`). La app no libera el id de una macro que ya no usa ninguna tecla, así que con el uso los ids pasaban de 63, y una secuencia rehecha a partir de ahí se quedaba tachada sin la app aunque solo llevara teclas, clics y esperas. Las secuencias se mudan en la Flash a 896-1024 KB; las de la 4.13 se traen solas al arrancar | Orvy 1.1.3 o posterior para que la app lo ofrezca como actualización; cualquiera para usarlo (la app lee el tope del handshake) |
