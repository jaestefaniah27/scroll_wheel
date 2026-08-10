# TODO — Rediseño "Apple-like" de OrbyGUI

Objetivo: aplicar la filosofía de diseño del plugin `frontend-design` (claridad,
jerarquía tipográfica fuerte, espaciado generoso, microinteracciones sutiles) a
toda la app. Sin framework, sin TypeScript — se queda en CSS + JS vanilla como
ahora.

## Restricciones que ya existen y hay que respetar

- **Sin `backdrop-filter` dentro de contenedores con scroll.** Se quitó a
  propósito (ver comentario en `src/styles/index.css` sobre `.glass-panel`):
  descompone el scroll compuesto y con rueda de alta resolución se ve a
  tirones. Si el nuevo diseño quiere efecto cristal, tiene que ser un color
  compuesto de antemano (como `--bg-panel-solid`), no un filtro en vivo.
- **Sin fuentes web.** La tipografía es pila del sistema
  (`Segoe UI Variable Text` → `Segoe UI` → `system-ui`...). Antes se traía
  Inter de Google Fonts y la app se quedaba sin tipografía sin conexión. Si el
  rediseño quiere una tipografía con más carácter, tiene que ser una que ya
  traiga Windows o quedar embebida en el paquete (`.woff2` local), nunca CDN.
- **Todo en castellano** (comentarios, textos de UI).
- **`prefers-reduced-motion`**: si se añaden microinteracciones/transiciones,
  respetarlo.

## Estado actual

Paleta violeta/índigo ya definida en `:root` (`src/styles/index.css`), con
tokens de color, sombra y radio. Sirve de punto de partida — no hace falta
inventar paleta desde cero, sí revisar si aguanta la filosofía nueva
(contraste, jerarquía, uso consistente).

Tamaño del CSS y las vistas a día de este TODO:

| Fichero | Líneas |
|---|---|
| `src/styles/index.css` | ~1500 |
| `src/views/profiles.js` | ~1650 |
| `src/views/oled.js` | ~1080 |
| `src/views/auto.js` | ~450 |
| `src/views/settings.js` | ~590 |
| `src/views/dashboard.js` | ~210 |
| `src/views/console.js` | ~100 |

## Fases

### Fase 1 — Sistema de diseño
- [ ] Revisar/ampliar tokens en `:root`: tipografía (escala, pesos), espaciado
      (escala de 4/8px), radios, sombras, duraciones de transición.
- [ ] Definir estilos base de componentes reutilizados en todas las vistas:
      botones, inputs, cards (`.glass-panel`), tabs, badges/pills, tooltips.
- [ ] Decidir cómo tratar estados (hover/active/disabled/focus-visible) de
      forma consistente — hoy están resueltos vista a vista.

### Fase 2 — `index.css`
- [ ] Reescribir componentes base sobre el sistema de la Fase 1.
- [ ] Revisar sidebar (`--sidebar-w` / `--sidebar-w-open`) y barra de título.

### Fase 3 — Vistas (de más simple a más compleja)
- [ ] `console.js` (~100 líneas)
- [ ] `dashboard.js` (~210 líneas)
- [ ] `settings.js` (~590 líneas)
- [ ] `auto.js` (~450 líneas)
- [ ] `oled.js` (~1080 líneas) — editor de framebuffer, grids, preview. Cuidado
      al tocar el layout: hay canvas/bitmaps posicionados a medida.
- [ ] `profiles.js` (~1650 líneas) — la vista más grande, listas, editor de
      teclas, drag&drop probablemente. Dejarla para el final con el sistema ya
      maduro en el resto.

### Fase 4 — Verificación
- [ ] `npm run dev`, pasar por las 6 vistas, comprobar look en cada una.
- [ ] Comprobar que el scroll con rueda de alta resolución sigue fluido en
      paneles largos (motivo original de quitar `backdrop-filter`).
- [ ] Arranque sin conexión: confirmar que la tipografía no depende de red.
- [ ] Repasar `prefers-reduced-motion` si se añadieron transiciones.

## Notas
- No cambia lógica en ningún punto de esta tarea — es solo presentación. Si al
  tocar una vista se ve necesario tocar JS de comportamiento, es un cambio
  aparte, no mezclar en el mismo commit.
- Ir marcando checkboxes en este fichero según se completen fases/vistas.
