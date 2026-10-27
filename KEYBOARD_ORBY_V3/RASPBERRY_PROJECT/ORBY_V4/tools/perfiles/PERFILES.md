# Biblioteca de perfiles

Generado por `tools/perfiles/build.py` a partir de `biblioteca.py`. No se edita a mano.

Teclas en el orden del teclado: la rejilla 3×3 y la 11 (abajo, en el centro).
«Seq:» es una secuencia de teclas, que el teclado reproduce él solo.

## Altium Designer (`Altium`)

Esquemático, rutado de PCB y vista/fabricación, una página cada uno.

Se activa solo con: `x2.exe`

### Esquemático

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Cable | `Ctrl+W` | No ERC | `Seq:P,V,N` |
| 2 | Etiqueta red | `Seq:P,N` | Anotar | `Seq:T,A,A` |
| 3 | Componente | `Seq:P,P` | Compilar | `Seq:C,C` |
| 4 | Alimentación | `Seq:P,O` | Similares | `Shift+F` |
| 5 | Puerto | `Seq:P,R` | Propiedades | `F11` |
| 6 | Bus | `Seq:P,B` | Sel. cruzada | `Ctrl+Shift+X` |
| 7 | Deshacer | `Ctrl+Z` | Espejo X | `X` |
| 8 | Girar | `Space` | Limpiar | `Shift+C` |
| 9 | Rehacer | `Ctrl+Y` | Espejo Y | `Y` |
| 11 | Guardar | `Ctrl+S` | Actualizar PCB | `Seq:D,U` |

Mando derecho: PageUp / PageDown, clic: Ctrl+PageDown. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+Tab / Ctrl+Shift+Tab.

### PCB

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Rutar | `Ctrl+W` | Polígono | `Seq:P,G` |
| 2 | Vía | `Seq:P,V` | Relleno | `Seq:P,F` |
| 3 | Par dif. | `Seq:P,I` | Componente | `Seq:P,C` |
| 4 | Capa ant. | `KP-` | Modo rutado | `Shift+R` |
| 5 | Capa única | `Shift+S` | Ancho | `Shift+W` |
| 6 | Capa sig. | `KP+` | Tamaño vía | `Shift+V` |
| 7 | Deshacer | `Ctrl+Z` | Cambiar cara | `L` |
| 8 | Girar | `Space` | Ajuste long. | `Seq:T,R` |
| 9 | Rehacer | `Ctrl+Y` | Limpiar | `Shift+C` |
| 11 | Guardar | `Ctrl+S` | Medir | `Ctrl+M` |

Mando derecho: PageUp / PageDown, clic: Ctrl+PageDown. Rueda: desplazar. Con SUPER, mando derecho: KP+ / KP-.

### Vista y salida

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Vista 2D | `2` | Smart PDF | `Seq:F,M` |
| 2 | Vista 3D | `3` | Materiales | `Seq:R,I` |
| 3 | Voltear placa | `Seq:V,B` | Act. esquema | `Seq:D,U` |
| 4 | Capas | `L` | Doc. anterior | `Ctrl+Shift+Tab` |
| 5 | Encuadrar | `Ctrl+PageDown` | Rejilla | `G` |
| 6 | Unidades | `Q` | Doc. siguiente | `Ctrl+Tab` |
| 7 | Repoblar | `Seq:T,G,A` | Similares | `Shift+F` |
| 8 | DRC | `Seq:T,D` | Propiedades | `F11` |
| 9 | Lágrimas | `Seq:T,E` | Todo | `Ctrl+A` |
| 11 | Compilar | `Seq:C,C` | Guardar | `Ctrl+S` |

Mando derecho: PageUp / PageDown, clic: Ctrl+PageDown. Rueda: desplazar. Con SUPER, mando derecho: PageUp / PageDown.

## KiCad (`KiCad`)

Esquemático y PCB de KiCad 8/9.

Se activa solo con: `eeschema.exe`, `pcbnew.exe`, `kicad.exe`

### Esquemático

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Cable | `W` | Etiq. global | `Ctrl+L` |
| 2 | Etiqueta red | `L` | Etiq. jerárq. | `H` |
| 3 | Componente | `A` | Bus | `B` |
| 4 | Alimentación | `P` | Mover | `M` |
| 5 | Sin conexión | `Q` | Propiedades | `E` |
| 6 | Unión | `J` | Arrastrar | `G` |
| 7 | Deshacer | `Ctrl+Z` | Espejo X | `X` |
| 8 | Girar | `R` | Repetir | `Insert` |
| 9 | Rehacer | `Ctrl+Y` | Espejo Y | `Y` |
| 11 | Guardar | `Ctrl+S` | Actualizar PCB | `F8` |

Mando derecho: F1 / F2, clic: Home. Rueda: desplazar. Con SUPER, mando derecho: F1 / F2.

### PCB

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Rutar | `X` | Rellenar zonas | `B` |
| 2 | Vía | `Ctrl+Shift+V` | Vaciar zonas | `Ctrl+B` |
| 3 | Par dif. | `6` | Huella | `A` |
| 4 | Capa ant. | `KP-` | Ancho | `W` |
| 5 | Cambiar cara | `F` | Arrastrar | `D` |
| 6 | Capa sig. | `KP+` | Sel. conexión | `U` |
| 7 | Deshacer | `Ctrl+Z` | Alto contraste | `Ctrl+H` |
| 8 | Girar | `R` | Medir | `Ctrl+Shift+M` |
| 9 | Rehacer | `Ctrl+Y` | Vista 3D | `Alt+3` |
| 11 | Guardar | `Ctrl+S` | Encuadrar | `Home` |

Mando derecho: F1 / F2, clic: Home. Rueda: desplazar. Con SUPER, mando derecho: KP+ / KP-.

## Fusion 360 (`Fusion`)

Boceto y modelado de sólidos.

Se activa solo con: `fusion360.exe`

### Boceto

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Línea | `L` | Buscar orden | `S` |
| 2 | Rectángulo | `R` | Medir | `I` |
| 3 | Círculo | `C` | Mostrar | `V` |
| 4 | Cota | `D` | Aspecto | `A` |
| 5 | Recortar | `T` | Encuadrar | `F6` |
| 6 | Desfase | `O` | Recalcular | `Ctrl+B` |
| 7 | Deshacer | `Ctrl+Z` | Deshacer | `Ctrl+Z` |
| 8 | Construcción | `X` | Borrar | `Delete` |
| 9 | Rehacer | `Ctrl+Y` | Rehacer | `Ctrl+Y` |
| 11 | Proyectar | `P` | Guardar | `Ctrl+S` |

Mando derecho: desplazar / desplazar, clic: F6. Rueda: desplazar. Con SUPER, mando derecho: desplazar / desplazar.

### Sólidos

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Extruir | `E` | Buscar orden | `S` |
| 2 | Presionar | `Q` | Medir | `I` |
| 3 | Empalme | `F` | Mostrar | `V` |
| 4 | Agujero | `H` | Aspecto | `A` |
| 5 | Mover | `M` | Encuadrar | `F6` |
| 6 | Unión | `J` | Recalcular | `Ctrl+B` |
| 7 | Deshacer | `Ctrl+Z` | Deshacer | `Ctrl+Z` |
| 8 | Medir | `I` | Borrar | `Delete` |
| 9 | Rehacer | `Ctrl+Y` | Rehacer | `Ctrl+Y` |
| 11 | Encuadrar | `F6` | Guardar | `Ctrl+S` |

Mando derecho: desplazar / desplazar, clic: F6. Rueda: desplazar. Con SUPER, mando derecho: desplazar / desplazar.

## LTspice (`LTspice`)

Dibujo del esquema y componentes básicos.

Se activa solo con: `ltspice.exe`, `xviix64.exe`

### Esquema

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Cable | `F3` | Diodo | `D` |
| 2 | Componente | `F2` | Directiva | `S` |
| 3 | Etiqueta red | `F4` | Texto | `T` |
| 4 | Resistencia | `R` | Mover | `F7` |
| 5 | Condensador | `C` | Arrastrar | `F8` |
| 6 | Bobina | `L` | Duplicar | `F6` |
| 7 | Deshacer | `F9` | Espejo | `Ctrl+E` |
| 8 | Girar | `Ctrl+R` | Borrar | `F5` |
| 9 | Rehacer | `Shift+F9` | Encuadrar | `Space` |
| 11 | Masa | `G` | Guardar | `Ctrl+S` |

Mando derecho: desplazar / desplazar, clic: Space. Rueda: desplazar. Con SUPER, mando derecho: desplazar / desplazar.

## Visual Studio Code (`VSCode`)

Edición, paneles y depuración.

Se activa solo con: `code.exe`

### Edición

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Comandos | `Ctrl+Shift+P` | Formatear | `Shift+Alt+F` |
| 2 | Ir a archivo | `Ctrl+P` | Renombrar | `F2` |
| 3 | Buscar todo | `Ctrl+Shift+F` | Reemplazar | `Ctrl+H` |
| 4 | Multicursor | `Ctrl+D` | Definición | `F12` |
| 5 | Subir línea | `Alt+Up` | Referencias | `Shift+F12` |
| 6 | Duplicar línea | `Shift+Alt+Down` | Atrás | `Alt+Left` |
| 7 | Deshacer | `Ctrl+Z` | Borrar línea | `Ctrl+Shift+K` |
| 8 | Bajar línea | `Alt+Down` | Arreglo rápido | `Ctrl+Punto` |
| 9 | Rehacer | `Ctrl+Y` | Ir a símbolo | `Ctrl+Shift+O` |
| 11 | Guardar | `Ctrl+S` | Comentar | `Seq:Ctrl+K,Ctrl+C` |

Mando derecho: Ctrl+PageDown / Ctrl+PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+PageDown / Ctrl+PageUp.

### Paneles

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Explorador | `Ctrl+Shift+E` | Nuevo archivo | `Ctrl+N` |
| 2 | Git | `Ctrl+Shift+G` | Abrir carpeta | `Seq:Ctrl+K,Ctrl+O` |
| 3 | Extensiones | `Ctrl+Shift+X` | Ajustes | `Ctrl+Coma` |
| 4 | Barra lateral | `Ctrl+B` | Ajuste línea | `Alt+Z` |
| 5 | Panel | `Ctrl+J` | Vista previa | `Ctrl+Shift+V` |
| 6 | Problemas | `Ctrl+Shift+M` | Cerrar todo | `Seq:Ctrl+K,W` |
| 7 | Pestaña ant. | `Ctrl+PageUp` | Plegar todo | `Seq:Ctrl+K,Ctrl+0` |
| 8 | Cerrar pest. | `Ctrl+W` | Guardar todo | `Seq:Ctrl+K,S` |
| 9 | Pestaña sig. | `Ctrl+PageDown` | Desplegar | `Seq:Ctrl+K,Ctrl+J` |
| 11 | Modo zen | `Seq:Ctrl+K,Z` | Reabrir | `Ctrl+Shift+T` |

Mando derecho: Ctrl+PageDown / Ctrl+PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+PageDown / Ctrl+PageUp.

### Depuración

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Depurar | `F5` | Definición | `F12` |
| 2 | Ejecutar | `Ctrl+F5` | Referencias | `Shift+F12` |
| 3 | Detener | `Shift+F5` | Ir a símbolo | `Ctrl+Shift+O` |
| 4 | Saltar | `F10` | Atrás | `Alt+Left` |
| 5 | Entrar | `F11` | Problemas | `Ctrl+Shift+M` |
| 6 | Salir | `Shift+F11` | Adelante | `Alt+Right` |
| 7 | Punto parada | `F9` | Deshacer | `Ctrl+Z` |
| 8 | Reiniciar | `Ctrl+Shift+F5` | Git | `Ctrl+Shift+G` |
| 9 | Compilar | `Ctrl+Shift+B` | Rehacer | `Ctrl+Y` |
| 11 | Panel | `Ctrl+J` | Guardar | `Ctrl+S` |

Mando derecho: Ctrl+PageDown / Ctrl+PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+PageDown / Ctrl+PageUp.

## Windows (`Windows`)

Ventanas y escritorios virtuales; música y utilidades del sistema.

### Ventanas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Escritorio | `Win+D` | Escrit. ant. | `Win+Ctrl+Left` |
| 2 | Vista tareas | `Win+Tab` | Nuevo escrit. | `Win+Ctrl+D` |
| 3 | Archivos | `Win+E` | Escrit. sig. | `Win+Ctrl+Right` |
| 4 | A la izquierda | `Win+Left` | Portapapeles | `Win+V` |
| 5 | Maximizar | `Win+Up` | Emojis | `Win+Punto` |
| 6 | A la derecha | `Win+Right` | Bloquear | `Win+L` |
| 7 | Otra pantalla | `Win+Shift+Left` | Ajustes | `Win+I` |
| 8 | Minimizar | `Win+Down` | Tareas | `Ctrl+Shift+Esc` |
| 9 | Cerrar | `Alt+F4` | Cerrar escrit. | `Win+Ctrl+F4` |
| 11 | Captura | `Win+Shift+S` | Buscar | `Win+S` |

Mando derecho: BrightUp / BrightDown, clic: —. Rueda: desplazar. Con SUPER, mando derecho: BrightUp / BrightDown.

### Multimedia

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Anterior | `Media:Prev` | Grabar | `Win+Alt+R` |
| 2 | Reproducir | `Media:Play` | Game Bar | `Win+G` |
| 3 | Siguiente | `Media:Next` | Captura total | `Win+PrintScreen` |
| 4 | Silencio | `Media:Mute` | Dictado | `Win+H` |
| 5 | Detener | `Media:Stop` | Notificaciones | `Win+N` |
| 6 | Silenciar mic | `Win+Alt+K` | Ajustes ráp. | `Win+A` |
| 7 | Navegador | `Media:Browser` | Ejecutar | `Win+R` |
| 8 | Correo | `Media:Mail` | Widgets | `Win+W` |
| 9 | Calculadora | `Media:Calc` | Ajustes | `Win+I` |
| 11 | Proyectar | `Win+P` | Buscar | `Win+S` |

Mando derecho: BrightUp / BrightDown, clic: —. Rueda: desplazar. Con SUPER, mando derecho: BrightUp / BrightDown.

## Navegador (Chrome, Edge, Firefox) (`Web`)

Pestañas y navegación; y una página para YouTube.

Se activa solo con: `chrome.exe`, `msedge.exe`, `firefox.exe`, `brave.exe`, `opera.exe`

### Pestañas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Pestaña ant. | `Ctrl+Shift+Tab` | Ventana nueva | `Ctrl+N` |
| 2 | Nueva pestaña | `Ctrl+T` | Incógnito | `Ctrl+Shift+N` |
| 3 | Pestaña sig. | `Ctrl+Tab` | Descargas | `Ctrl+J` |
| 4 | Atrás | `Alt+Left` | Historial | `Ctrl+H` |
| 5 | Recargar | `F5` | Favorito | `Ctrl+D` |
| 6 | Adelante | `Alt+Right` | Completa | `F11` |
| 7 | Reabrir pest. | `Ctrl+Shift+T` | Zoom 100 % | `Ctrl+0` |
| 8 | Dirección | `Ctrl+L` | DevTools | `F12` |
| 9 | Cerrar pest. | `Ctrl+W` | Recargar todo | `Ctrl+Shift+R` |
| 11 | Buscar | `Ctrl+F` | Imprimir | `Ctrl+P` |

Mando derecho: Ctrl + rueda / Ctrl + rueda, clic: Ctrl+0. Rueda: desplazar. Con SUPER, mando derecho: Ctrl + rueda / Ctrl + rueda.

### YouTube

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | -10 s | `J` | Pestaña ant. | `Ctrl+Shift+Tab` |
| 2 | Reproducir | `K` | Minirrepr. | `I` |
| 3 | +10 s | `L` | Pestaña sig. | `Ctrl+Tab` |
| 4 | Anterior | `Shift+P` | Atrás | `Alt+Left` |
| 5 | Completa | `F` | Recargar | `F5` |
| 6 | Siguiente | `Shift+N` | Adelante | `Alt+Right` |
| 7 | Silenciar | `M` | Historial | `Ctrl+H` |
| 8 | Subtítulos | `C` | Dirección | `Ctrl+L` |
| 9 | Al inicio | `0` | Cerrar pest. | `Ctrl+W` |
| 11 | Modo cine | `T` | Reabrir pest. | `Ctrl+Shift+T` |

Mando derecho: Left / Right, clic: K. Rueda: desplazar. Con SUPER, mando derecho: Left / Right.

## Microsoft Excel (`Excel`)

Edición de celdas; filas, columnas y hojas.

Se activa solo con: `excel.exe`

### Edición

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Copiar | `Ctrl+C` | Formato celda | `Ctrl+1` |
| 2 | Pegar | `Ctrl+V` | Negrita | `Ctrl+N` |
| 3 | Pegado esp. | `Ctrl+Alt+V` | Filtro | `Ctrl+Shift+L` |
| 4 | Editar celda | `F2` | Salto en celda | `Alt+Enter` |
| 5 | Rellenar abajo | `Ctrl+J` | Nota | `Shift+F2` |
| 6 | Fijar ref. | `F4` | Rellenar dcha. | `Ctrl+D` |
| 7 | Deshacer | `Ctrl+Z` | Buscar | `Ctrl+B` |
| 8 | Autosuma | `Alt+Shift+0` | Recalcular | `F9` |
| 9 | Rehacer | `Ctrl+Y` | Reemplazar | `Ctrl+L` |
| 11 | Guardar | `Ctrl+G` | Gráfico | `Alt+F1` |

Mando derecho: Ctrl+PageDown / Ctrl+PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+PageDown / Ctrl+PageUp.

### Filas y hojas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Fila | `Shift+Space` | Al inicio | `Ctrl+Home` |
| 2 | Insertar | `Ctrl+KP+` | Todo | `Ctrl+E` |
| 3 | Columna | `Ctrl+Space` | Imprimir | `Ctrl+P` |
| 4 | Ocultar fila | `Ctrl+9` | Copiar | `Ctrl+C` |
| 5 | Eliminar | `Ctrl+KP-` | Pegar | `Ctrl+V` |
| 6 | Ir a | `Ctrl+I` | Cortar | `Ctrl+X` |
| 7 | Hoja ant. | `Ctrl+PageUp` | Deshacer | `Ctrl+Z` |
| 8 | Hoja nueva | `Shift+F11` | Buscar | `Ctrl+B` |
| 9 | Hoja sig. | `Ctrl+PageDown` | Rehacer | `Ctrl+Y` |
| 11 | Al final | `Ctrl+End` | Guardar | `Ctrl+G` |

Mando derecho: Ctrl+PageDown / Ctrl+PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+PageDown / Ctrl+PageUp.

## Microsoft Word (`Word`)

Formato de texto; documento y revisión.

Se activa solo con: `winword.exe`

### Texto

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Negrita | `Ctrl+N` | Copiar formato | `Ctrl+Shift+C` |
| 2 | Cursiva | `Ctrl+K` | Pegar formato | `Ctrl+Shift+V` |
| 3 | Subrayado | `Ctrl+S` | Quitar formato | `Ctrl+Space` |
| 4 | Izquierda | `Ctrl+Q` | Título 1 | `Ctrl+Alt+1` |
| 5 | Centrar | `Ctrl+T` | Título 2 | `Ctrl+Alt+2` |
| 6 | Derecha | `Ctrl+D` | Título 3 | `Ctrl+Alt+3` |
| 7 | Deshacer | `Ctrl+Z` | Hipervínculo | `Ctrl+Alt+K` |
| 8 | Justificar | `Ctrl+J` | Salto página | `Ctrl+Enter` |
| 9 | Rehacer | `Ctrl+Y` | Buscar | `Ctrl+B` |
| 11 | Guardar | `Ctrl+G` | Reemplazar | `Ctrl+L` |

Mando derecho: BrightUp / BrightDown, clic: —. Rueda: desplazar. Con SUPER, mando derecho: BrightUp / BrightDown.

### Documento

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Nuevo | `Ctrl+U` | Copiar | `Ctrl+C` |
| 2 | Abrir | `Ctrl+A` | Pegar | `Ctrl+V` |
| 3 | Imprimir | `Ctrl+P` | Cortar | `Ctrl+X` |
| 4 | Ortografía | `F7` | Negrita | `Ctrl+N` |
| 5 | Sinónimos | `Shift+F7` | Cursiva | `Ctrl+K` |
| 6 | Ir a | `Ctrl+I` | Subrayado | `Ctrl+S` |
| 7 | Al inicio | `Ctrl+Home` | Deshacer | `Ctrl+Z` |
| 8 | Todo | `Ctrl+E` | Guardar | `Ctrl+G` |
| 9 | Al final | `Ctrl+End` | Rehacer | `Ctrl+Y` |
| 11 | Cerrar doc. | `Ctrl+F4` | Buscar | `Ctrl+B` |

Mando derecho: BrightUp / BrightDown, clic: —. Rueda: desplazar. Con SUPER, mando derecho: BrightUp / BrightDown.

## Microsoft PowerPoint (`PPoint`)

Presentar con puntero y tinta; con SUPER, editar diapositivas.

Se activa solo con: `powerpnt.exe`

### Presentar

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Desde inicio | `F5` | Nueva diapo. | `Ctrl+M` |
| 2 | Desde actual | `Shift+F5` | Negrita | `Ctrl+N` |
| 3 | Moderador | `Alt+F5` | Cursiva | `Ctrl+K` |
| 4 | Anterior | `PageUp` | Izquierda | `Ctrl+Q` |
| 5 | Pantalla negra | `Punto` | Centrar | `Ctrl+T` |
| 6 | Siguiente | `PageDown` | Derecha | `Ctrl+D` |
| 7 | Láser | `Ctrl+L` | Deshacer | `Ctrl+Z` |
| 8 | Lápiz | `Ctrl+P` | Copiar formato | `Ctrl+Shift+C` |
| 9 | Borrar tinta | `E` | Rehacer | `Ctrl+Y` |
| 11 | Terminar | `Esc` | Guardar | `Ctrl+G` |

Mando derecho: PageDown / PageUp, clic: —. Rueda: desplazar. Con SUPER, mando derecho: PageDown / PageUp.

## Adobe Photoshop (`PShop`)

Herramientas, capas y ajustes de imagen.

Se activa solo con: `photoshop.exe`

### Herramientas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Mover | `V` | Pluma | `P` |
| 2 | Marco | `M` | Texto | `T` |
| 3 | Lazo | `L` | Forma | `U` |
| 4 | Selec. rápida | `W` | Degradado | `G` |
| 5 | Recortar | `C` | Sobreexponer | `O` |
| 6 | Cuentagotas | `I` | Trazado | `A` |
| 7 | Pincel | `B` | Mano | `H` |
| 8 | Borrador | `E` | Zoom | `Z` |
| 9 | Corrector | `J` | Rotar vista | `R` |
| 11 | Tampón | `S` | Máscara ráp. | `Q` |

Mando derecho: Ctrl+Mas / Ctrl+Menos, clic: Ctrl+0. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+Shift+Z / Ctrl+Z.

### Capas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Capa nueva | `Ctrl+Shift+N` | Deseleccionar | `Ctrl+D` |
| 2 | Duplicar | `Ctrl+J` | Invertir sel. | `Ctrl+Shift+I` |
| 3 | Combinar | `Ctrl+E` | Todo | `Ctrl+A` |
| 4 | Agrupar | `Ctrl+G` | Rellenar | `Shift+F5` |
| 5 | Máscara rec. | `Ctrl+Alt+G` | Intercambiar | `X` |
| 6 | Sellar visible | `Ctrl+Alt+Shift+E` | Colores def. | `D` |
| 7 | Deshacer | `Ctrl+Z` | Encuadrar | `Ctrl+0` |
| 8 | Transformar | `Ctrl+T` | 100 % | `Ctrl+1` |
| 9 | Rehacer | `Ctrl+Shift+Z` | Modo pantalla | `F` |
| 11 | Guardar | `Ctrl+S` | Guardar como | `Ctrl+Shift+S` |

Mando derecho: Ctrl+Mas / Ctrl+Menos, clic: Ctrl+0. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+Shift+Z / Ctrl+Z.

### Ajustes

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Niveles | `Ctrl+L` | Contraste auto. | `Ctrl+Alt+Shift+L` |
| 2 | Curvas | `Ctrl+M` | Color auto. | `Ctrl+Shift+B` |
| 3 | Tono/sat. | `Ctrl+U` | Blanco y negro | `Ctrl+Alt+Shift+B` |
| 4 | Equilibrio | `Ctrl+B` | Encuadrar | `Ctrl+0` |
| 5 | Invertir | `Ctrl+I` | 100 % | `Ctrl+1` |
| 6 | Desaturar | `Ctrl+Shift+U` | Transformar | `Ctrl+T` |
| 7 | Tamaño imagen | `Ctrl+Alt+I` | Deshacer | `Ctrl+Z` |
| 8 | Lienzo | `Ctrl+Alt+C` | Guardar | `Ctrl+S` |
| 9 | Tono auto. | `Ctrl+Shift+L` | Rehacer | `Ctrl+Shift+Z` |
| 11 | Exportar como | `Ctrl+Alt+Shift+W` | Guardar como | `Ctrl+Shift+S` |

Mando derecho: Ctrl+Mas / Ctrl+Menos, clic: Ctrl+0. Rueda: desplazar. Con SUPER, mando derecho: Ctrl+Shift+Z / Ctrl+Z.

## Adobe Premiere Pro (`Premier`)

Montaje en la línea de tiempo y herramientas.

Se activa solo con: `adobe premiere pro.exe`

### Montaje

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Cuchilla | `C` | Entrada | `I` |
| 2 | Selección | `V` | Marcar clip | `X` |
| 3 | Añadir corte | `Ctrl+K` | Salida | `O` |
| 4 | Recortar ant. | `Q` | Corte ant. | `Up` |
| 5 | Borrar rizo | `Shift+Delete` | Fot. coincid. | `F` |
| 6 | Recortar sig. | `W` | Corte sig. | `Down` |
| 7 | Deshacer | `Ctrl+Z` | Vincular | `Ctrl+L` |
| 8 | Marcador | `M` | Ajustar | `S` |
| 9 | Rehacer | `Ctrl+Shift+Z` | Velocidad | `Ctrl+R` |
| 11 | Reproducir | `Space` | Renderizar | `Enter` |

Mando derecho: Right / Left, clic: Space. Rueda: desplazar. Con SUPER, mando derecho: Down / Up.

### Herramientas

| Tecla | Normal | Atajo | SUPER | Atajo |
|---|---|---|---|---|
| 1 | Edición rizo | `B` | Trans. vídeo | `Ctrl+D` |
| 2 | Rotación | `N` | Trans. audio | `Ctrl+Shift+D` |
| 3 | Estirar | `R` | Transiciones | `Shift+D` |
| 4 | Desplazar | `Y` | Exportar fot. | `Ctrl+Shift+E` |
| 5 | Deslizar | `U` | Exportar | `Ctrl+M` |
| 6 | Sel. pista | `A` | Guardar | `Ctrl+S` |
| 7 | Pluma | `P` | Deshacer | `Ctrl+Z` |
| 8 | Mano | `H` | Reproducir | `Space` |
| 9 | Texto | `T` | Rehacer | `Ctrl+Shift+Z` |
| 11 | Zoom | `Z` | Renderizar | `Enter` |

Mando derecho: Right / Left, clic: Space. Rueda: desplazar. Con SUPER, mando derecho: Down / Up.

