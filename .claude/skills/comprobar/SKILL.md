---
name: comprobar
description: Pasa de una vez todas las pruebas de ORBY que corren en el PC sin teclado (tests de la app OrbyGUI, tests C++ del firmware, iconos regenerados, descriptor HID, mapa de la Flash…) y el estado de git. Úsala antes de un commit, de un push o de publicar, y cuando pidan pasar los tests, comprobar que todo está bien o ver si algo se ha roto.
---

# Comprobar ORBY antes de subir

```powershell
powershell -ExecutionPolicy Bypass -File D:\ORVY\scroll_wheel\.claude\skills\comprobar\comprobar.ps1          # todo (~1-2 min)
powershell -ExecutionPolicy Bypass -File D:\ORVY\scroll_wheel\.claude\skills\comprobar\comprobar.ps1 -SoloApp # solo OrbyGUI (~10 s)
```

Sale con código 0 si todo está en verde y termina con un resumen `[ OK ]` / `[FALLO]`.

## Qué pasa

1. **App:** `node --test --test-reporter=tap` sobre `OrbyGUI/test/*.test.mjs`. Si algo falla,
   imprime el `not ok` con su contexto.
2. **Firmware en el PC:** `ORBY_V4/tools/test/run.ps1`. Hace todo esto:
   - tests C++ (`test_hid_out`, `test_wheel`, `test_ui`, `test_anim`, `test_macro_store`);
   - `oled_ui/build.py --check`, que comprueba que los iconos están regenerados;
   - `perfiles/build.py --check`;
   - que la huella de configuración da lo mismo en C++ y en JS;
   - el descriptor HID;
   - que firmware y app coinciden en las teclas de sistema;
   - el mapa de la Flash.
3. **git:** `fetch` y `status -sb`, para ver si vas por delante o por detrás de `origin/main`.

No compila el firmware de la Pico ni la app de Tauri. Para eso, mira la skill `publicar`.

## Por qué el script y no los comandos sueltos

- **`npm test` está roto con Node 24** (`--experimental-default-type`). Por eso se usa
  `node --test` directamente.
- **g++ falla en silencio** («No compila.», sin ningún error) si otra carpeta del PATH va antes
  que `C:\msys64\mingw64\bin`, como pasa en Git Bash: carga DLLs que no son las suyas. El
  script pone msys64 al principio.
- **`run.ps1` recompila los `tools/test/*.exe`** y los deja modificados en git. El script
  restaura los que estaban limpios antes de empezar, para que no acaben en un commit.

## Si algo falla

- **`build.py --check`:** se ha tocado `tools/oled_ui` sin regenerar. Lanza
  `python ORBY_V4/tools/oled_ui/build.py` y sube también los ficheros generados
  (`include/oled_ui_data.h` y `OrbyGUI/src/oled-ui-data.mjs`).
- **`compat-tabla.test.mjs`:** el firmware de `orby_version.h` no tiene entrada en
  `OrbyGUI/src/compat.json`, o su `minApp` es mayor que la versión de `package.json`.
- **`games-juegos.test.mjs` («termina y dibuja sin romperse»):** un juego no acaba con teclas
  al azar. Mira la skill `nuevo-juego`.
- **`[behind N]` en git:** haz `git pull --rebase` antes de seguir. Hay commits en remoto que
  no tienes.
