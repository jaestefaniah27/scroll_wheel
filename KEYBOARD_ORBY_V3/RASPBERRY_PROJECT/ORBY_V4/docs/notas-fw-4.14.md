# Firmware 4.14

## Las secuencias vuelven a ir sin la app

Una secuencia de solo teclas, clics y esperas la reproduce el propio teclado,
sin la app abierta. Pero el teclado solo tenía hueco para 64, numeradas por la
app, y la app no reaprovecha el número de una macro que ya no se usa: con el
tiempo los números pasaban de 63, y una secuencia rehecha a partir de ahí se
quedaba en el PC. Sin la app, su pantalla salía tachada y la tecla no hacía nada.

Ahora caben **255**, el máximo que permite el formato de las teclas. La app lo
lee del teclado al conectar, así que en cuanto se conecta una vez sube las que
faltaban y vuelven a funcionar solas.

Además, al guardar en la Flash solo se reescribe la parte de las secuencias que
ha cambiado: menos tiempo con el teclado parado y menos desgaste.

Las secuencias que ya tenía el teclado se conservan: al arrancar por primera vez
con este firmware se traen de donde estaban.
