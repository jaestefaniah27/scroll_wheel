# Biblioteca de perfiles

Perfiles listos para usar, que OrbyGUI instala desde **Perfiles → Biblioteca**:

| Grupo | Perfiles |
|---|---|
| Ingeniería | Altium (esquemático / PCB / vista y salida), KiCad, Fusion 360, LTspice |
| Programación | VS Code (edición / paneles / depuración), Windows, navegador (pestañas / YouTube) |
| Office | Excel, Word, PowerPoint — **en español** |
| Creativos | Photoshop (herramientas / capas / ajustes), Premiere Pro (montaje / herramientas) |

Cada tecla de cada página, con su atajo, está en [PERFILES.md](PERFILES.md).

## Cómo están pensados

- **Una página por tarea.** Lo que se usa junto va junto: en Altium, el
  esquemático en una página y el rutado en otra. Un toque a la tecla de menú
  pasa a la siguiente.
- **Capa normal para lo de cada momento, SUPER para lo de cada rato.**
- **Mismas posiciones en todos los perfiles** donde tiene sentido: Deshacer y
  Rehacer en las esquinas de abajo (teclas 7 y 9), y la 11 para la orden que más
  se repite (casi siempre Guardar).
- **Mandos:** el izquierdo es el volumen en todos. El derecho hace en cada
  programa lo que más se gira: zoom en Altium, KiCad y Photoshop; pestañas en VS Code
  y el navegador; hojas en Excel; fotograma a fotograma en Premiere. Con SUPER cambia
  (capas del PCB, historial de Photoshop, saltar de corte en corte...).
- **Office y el teclado, en español.** Ctrl+G guarda, Ctrl+N es negrita, Ctrl+B
  busca. Los atajos con signos solo se usan cuando el signo cae en la misma tecla
  con el teclado español.
- **Secuencias.** Altium funciona con aceleradores de menú («P, V» coloca una vía)
  y VS Code con acordes (Ctrl+K, Ctrl+C). Se instalan como secuencias que el
  teclado reproduce él solo, sin depender de que OrbyGUI esté abierta.
- **Cambio automático.** Al instalar uno, OrbyGUI crea la regla para que se ponga
  solo al abrir su programa (si no había ya una para ese programa).

Los iconos de las pantallas los pone el firmware a partir de las etiquetas
(`tools/oled_ui/catalog_apps.py`): instalar un perfil no sube ni un bitmap.

## Cambiar un perfil o añadir otro

1. Edita `biblioteca.py` (cómo se escribe cada página: `modelo.py`).
2. `python3 tools/perfiles/build.py`. Falla si una etiqueta pasa de 7 caracteres,
   si se ocupa la tecla de menú o la de SUPER, o si una etiqueta no tiene icono.
3. Si había etiquetas sin icono: añádelas a `tools/oled_ui/catalog_apps.py` (y el
   dibujo a `icons_apps.py` si hace falta uno nuevo), `python3 tools/oled_ui/build.py`
   y vuelve a compilar el firmware.

Salidas, las dos van al repositorio: `OrbyGUI/src/profile-library.json` y `PERFILES.md`.

## Lo que no se ha podido probar

Los atajos salen de la documentación de cada programa, pero no se han probado
uno a uno con el programa delante. Los que más conviene comprobar:

- Office en español: Ctrl+J / Ctrl+D (rellenar abajo / a la derecha en Excel),
  Ctrl+Alt+K (hipervínculo en Word) y Ctrl+M (nueva diapositiva en PowerPoint).
- Las secuencias de Altium dependen de los aceleradores de menú de la versión
  instalada; si alguna abre otro menú, se cambia la letra en `biblioteca.py`.
- KiCad 8/9 con los atajos por defecto; con un esquema de teclas propio, no.
