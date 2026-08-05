# Lámpara LampDesk — complemento para OrbyGUI

Pone la lámpara del escritorio en una tecla o en un mando del Orby: encender,
apagar, alternar, y el brillo o el color en un encoder.

Este complemento **no** va dentro del instalador de OrbyGUI: la app se reparte
sin él y se instala aparte. Ver [`docs/PLUGINS.md`](../../docs/PLUGINS.md) para
escribir uno propio.

## Instalación

1. `npm run pack:plugins` en `OrbyGUI/` deja `release/plugins/lampdesk-1.0.0.zip`.
   (En desarrollo se puede instalar esta carpeta directamente, sin empaquetar.)
2. En OrbyGUI: **Ajustes → Complementos → Instalar complemento**.
3. En la tarjeta *Lámpara LampDesk* que aparece, pon su dirección y pulsa
   *Probar*.

## Ajustes

**Dirección de la lámpara** — dirección y puerto en la red, por defecto
`192.168.1.35:8080`. El puerto no es el 80 porque ese lo ocupa el servidor
HomeKit dentro de la propia lámpara.

Conviene fijarle reserva DHCP en el router: el nombre mDNS que publica lo pone
HomeSpan a partir del identificador del accesorio y cambia si se vuelve a
emparejar, así que no sirve como dirección estable.

## Acciones

| Acción | Dónde se asigna |
|---|---|
| Encender / apagar, Encender, Apagar | Una tecla (pestaña *Multimedia*) o la pulsación de un encoder |
| Brillo de la lámpara | El giro de un encoder o de la rueda |
| Color de la lámpara | El giro de un encoder o de la rueda |

Cada muesca mueve cinco puntos de cien y el signo lo pone el hueco —horario
sube, antihorario baja—, así que basta con elegir lo mismo en los dos sentidos.
Si el encoder está montado del revés, *Invertir giro* le da la vuelta al par.

Los valores van en el mismo 0-100 que enseña la app Casa, así que los tres
mandos —el encoder, el panel de la bandeja y el iPhone— marcan siempre lo mismo.
Los incrementos los resuelve la propia lámpara (`dbrightness`, `dcolor`), no el
PC: quien manda «sube cinco» no tiene por qué saber de cuánto se partía.

## Cómo funciona

Las órdenes salen del PC, no del teclado: el firmware avisa con `MACRO:<id>` y
OrbyGUI llama a la API HTTP de la lámpara, así que la app tiene que estar
abierta (basta con el icono de la bandeja).

Los incrementos de un giro rápido se agrupan y viajan en una sola petición cada
120 ms, porque el teclado manda un aviso **por muesca** y la ESP32 las atiende
de una en una. Encender y apagar no se agrupan: dos pulsaciones seguidas son
dos órdenes, y fundidas en una sola «alterna» se anularían entre ellas.

El firmware de la lámpara vive en `ESP-32_Projects/lampDesk`.
