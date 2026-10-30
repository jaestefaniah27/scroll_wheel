# Orvy 1.1.3

Reconoce el firmware **4.14** y lo instala sola, como las versiones anteriores.

## Novedades

- **Secuencias que van solas, aunque haya muchas macros:** con el firmware 4.14
  el teclado guarda 255 secuencias en vez de 64, y las que solo llevan teclas,
  clics y esperas vuelven a funcionar sin la app.
- **Limpieza de macros sin usar:** cambiar una tecla de pestaña, pegarla o
  instalar un perfil de la biblioteca dejaba atrás la macro de antes, y los
  números de macro solo crecían. Ahora, al conectar el teclado, la app retira
  las que no usa ninguna tecla, mando ni variación por app. Lo retirado se
  guarda en la configuración del PC y vuelve solo si alguna tecla lo vuelve a
  pedir (por ejemplo, al restaurar una copia de seguridad antigua).
- **Velocidad del menú:** la app leía mal la velocidad de la animación del menú
  que tenía guardada el teclado (se comía la primera cifra).

## Compatibilidad

Reconoce el firmware 4.14. Con uno anterior funciona igual que la 1.1.2.
