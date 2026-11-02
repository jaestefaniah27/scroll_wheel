# Vídeos de los consejos

Uno por consejo, con el id del consejo como nombre (`modo-ayuda.mp4`, `zoom-touchpad.mp4`...;
los ids están en `OrbyGUI/src/tips.mjs`). Un consejo sin vídeo sale con «Vídeo próximamente».

## Formato

- mp4, H.264, **sin audio**, de 5 a 10 segundos, plano fijo sobre el teclado.
- **Ancho ≤ 480 px y ≤ 300 KB.** La app los enseña pequeños (148 px) a propósito: la baja
  resolución no se nota y la descarga es mínima.
- Ejemplo: `ffmpeg -i bruto.mov -an -vf "scale=480:-2,fps=20" -c:v libx264 -crf 30 -preset slow -movflags +faststart modo-ayuda.mp4`

## Publicar uno

1. Copia el `.mp4` a esta carpeta.
2. `python tools/consejos/manifiesto.py` (desde `ORBY_V4/`) regenera `manifest.json` con los hashes y
   avisa si alguno pasa de 300 KB.
3. Commit y push a `main`. Las apps instaladas lo bajan en el siguiente arranque (una vez, a
   `%APPDATA%\OrbyGUI\tips\`); la WebGUI lo ve directo desde GitHub.
