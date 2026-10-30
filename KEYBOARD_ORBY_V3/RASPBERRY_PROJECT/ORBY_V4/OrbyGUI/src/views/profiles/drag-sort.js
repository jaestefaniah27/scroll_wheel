// Reordenar arrastrando, para los pasos de una secuencia (lista vertical) y
// las teclas de la réplica (rejilla de celdas iguales).
//
// No se usa el arrastre nativo de HTML5: en WebView2 enseña una imagen
// fantasma semitransparente que no se puede animar, y no deja que los demás
// elementos se aparten mientras tanto. Aquí se mueve el propio elemento con
// `transform` y los demás se desplazan a su hueco nuevo con una transición,
// como en la pantalla de inicio del iPhone. El DOM no se toca hasta soltar:
// entonces `onDrop` cambia los datos y quien llama repinta, y como cada
// elemento ya está dibujado justo donde va a quedar, el repintado no se nota.
//
// Las posiciones se miden una vez al empezar. Lo que se desplace después
// (la página al hacer scroll) se compensa con el desplazamiento del
// contenedor que hace scroll, que es lo único que puede moverse durante el
// gesto.

const AUTOSCROLL_EDGE = 48;   // px desde el borde donde empieza el autoscroll
const AUTOSCROLL_SPEED = 14;  // px por fotograma en el mismo borde
const DROP_MS = 220;          // lo que tarda en asentarse al soltar

let active = null;

export function isDragging() {
  return active !== null;
}

// items: elementos que se pueden reordenar, en su orden actual (los que no
// se mueven —las teclas sin pantalla— no van en la lista y conservan su sitio).
// layout: 'list' (alturas distintas, en columna) | 'grid' (celdas iguales).
// onDrop(from, to): solo se llama si la posición cambia. onEnd: al acabar el
// gesto, se haya movido algo o no (también si se cancela).
export function startSortDrag({ items, fromIndex, event, layout, container, onDrop, onEnd }) {
  if (active) cancelDrag();
  const el = items[fromIndex];
  if (!el) return;

  const scroller = scrollParent(container);
  const rects = items.map((it) => it.getBoundingClientRect());
  // La réplica del Inicio va ampliada con un `scale` en un antepasado: lo que
  // se mueve el puntero en pantalla hay que pasarlo a píxeles del elemento.
  // Se mide en el contenedor y no en el elemento, que puede estar a medio
  // "levantarse" (is-pressing lo agranda).
  const scale = (container.offsetWidth && container.getBoundingClientRect().width / container.offsetWidth) || 1;
  const gap = layout === 'list' && rects.length > 1
    ? Math.max(0, rects[1].top - rects[0].bottom)
    : 0;

  active = {
    items, from: fromIndex, to: fromIndex, el, layout, container, scroller,
    rects, gap, scale, onDrop, onEnd,
    startX: event.clientX, startY: event.clientY,
    x: event.clientX, y: event.clientY,
    startScroll: scroller ? scroller.scrollTop : window.scrollY,
    raf: 0,
  };

  container.classList.add('is-sorting');
  el.classList.add('is-dragging');
  for (const it of items) if (it !== el) it.classList.add('is-shifting');
  document.body.classList.add('is-sort-dragging');

  window.addEventListener('pointermove', onMove, true);
  window.addEventListener('pointerup', onUp, true);
  window.addEventListener('pointercancel', onCancelEvent, true);
  window.addEventListener('keydown', onKey, true);
  // Si alguien repinta por debajo (llega un icono, el teclado cambia de
  // página), los elementos que se movían ya no existen: se corta el gesto.
  active.observer = new MutationObserver(() => {
    if (active && !document.contains(active.el)) cancelDrag();
  });
  active.observer.observe(document.body, { childList: true, subtree: true });

  tick();
}

// Decide si un pointerdown acaba en arrastre. Con `holdMs` hace falta dejarlo
// pulsado ese rato sin moverse (las teclas: un clic normal tiene que seguir
// seleccionándolas); sin él basta con moverlo unos píxeles (el asa de un
// paso, que no sirve para otra cosa). `pressEl` recibe `is-pressing`
// mientras se espera, para que se note que algo va a pasar.
export function armDrag(downEvent, { holdMs = 0, threshold = 4, pressEl = null, start }) {
  if (downEvent.button !== 0 || active) return;
  let last = downEvent;
  let timer = null;

  const cleanup = () => {
    clearTimeout(timer);
    pressEl?.classList.remove('is-pressing');
    window.removeEventListener('pointermove', move, true);
    window.removeEventListener('pointerup', cleanup, true);
    window.removeEventListener('pointercancel', cleanup, true);
  };
  const go = () => { cleanup(); start(last); };
  const move = (e) => {
    last = e;
    const far = Math.hypot(e.clientX - downEvent.clientX, e.clientY - downEvent.clientY) > threshold;
    if (!far) return;
    // Con espera, moverse antes de tiempo es que no quería arrastrar.
    if (holdMs) cleanup();
    else go();
  };

  window.addEventListener('pointermove', move, true);
  window.addEventListener('pointerup', cleanup, true);
  window.addEventListener('pointercancel', cleanup, true);
  if (holdMs) {
    pressEl?.classList.add('is-pressing');
    timer = setTimeout(go, holdMs);
  }
}

export function cancelDrag() {
  if (!active) return;
  const a = active;
  finish();
  for (const it of a.items) it.style.transform = '';
  a.onEnd?.();
}

function scrollParent(node) {
  for (let n = node?.parentElement; n && n !== document.body; n = n.parentElement) {
    const oy = getComputedStyle(n).overflowY;
    if ((oy === 'auto' || oy === 'scroll') && n.scrollHeight > n.clientHeight) return n;
  }
  return null;
}

function scrollOffset(a) {
  return (a.scroller ? a.scroller.scrollTop : window.scrollY) - a.startScroll;
}

function onMove(e) {
  if (!active) return;
  e.preventDefault();
  active.x = e.clientX;
  active.y = e.clientY;
}

function onKey(e) {
  if (e.key !== 'Escape') return;
  e.preventDefault();
  e.stopPropagation();
  cancelDrag();
}

function onCancelEvent() {
  cancelDrag();
}

// Un único bucle por fotograma: mueve el elemento, decide el hueco y hace
// autoscroll. Ir por eventos de puntero no basta para el autoscroll, que
// tiene que seguir aunque el ratón se quede quieto junto al borde.
function tick() {
  const a = active;
  if (!a) return;
  autoscroll(a);

  const dx = a.x - a.startX;
  const dy = a.y - a.startY + scrollOffset(a);
  a.el.style.transform = `translate(${dx / a.scale}px, ${dy / a.scale}px) scale(1.06)`;

  const to = a.layout === 'grid' ? gridTarget(a, dx, dy) : listTarget(a, dy);
  if (to !== a.to) {
    a.to = to;
    layoutOthers(a);
  }
  a.raf = requestAnimationFrame(tick);
}

function autoscroll(a) {
  const box = a.scroller ? a.scroller.getBoundingClientRect() : { top: 0, bottom: window.innerHeight };
  let v = 0;
  if (a.y < box.top + AUTOSCROLL_EDGE) v = -AUTOSCROLL_SPEED * (1 - (a.y - box.top) / AUTOSCROLL_EDGE);
  else if (a.y > box.bottom - AUTOSCROLL_EDGE) v = AUTOSCROLL_SPEED * (1 - (box.bottom - a.y) / AUTOSCROLL_EDGE);
  if (!v) return;
  v = Math.max(-AUTOSCROLL_SPEED, Math.min(AUTOSCROLL_SPEED, v));
  if (a.scroller) a.scroller.scrollTop += v;
  else window.scrollBy(0, v);
}

// En la lista, el hueco es el número de pasos cuyo centro queda por encima
// del centro del que se arrastra.
function listTarget(a, dy) {
  const r = a.rects[a.from];
  const center = r.top + r.height / 2 + dy;
  let to = 0;
  a.rects.forEach((q, i) => {
    if (i !== a.from && q.top + q.height / 2 < center) to++;
  });
  return to;
}

// En la rejilla, la celda cuyo centro está más cerca del de la tecla.
function gridTarget(a, dx, dy) {
  const r = a.rects[a.from];
  const cx = r.left + r.width / 2 + dx;
  const cy = r.top + r.height / 2 + dy;
  let best = a.from;
  let bestD = Infinity;
  a.rects.forEach((q, i) => {
    const d = (q.left + q.width / 2 - cx) ** 2 + (q.top + q.height / 2 - cy) ** 2;
    if (d < bestD) { bestD = d; best = i; }
  });
  return best;
}

// Orden que quedaría si se soltase ahora: índices originales.
function pendingOrder(a) {
  const order = a.items.map((_, i) => i);
  order.splice(a.from, 1);
  order.splice(a.to, 0, a.from);
  return order;
}

// Lleva cada elemento (menos el arrastrado) al sitio que ocuparía. En la
// rejilla, a la celda de su posición nueva; en la lista se apilan con sus
// alturas reales, porque un paso de texto mide más que uno de clic.
function targetOffsets(a) {
  const order = pendingOrder(a);
  const offs = new Array(a.items.length);
  if (a.layout === 'grid') {
    // Por centros: la caja del que se arrastra se midió agrandada.
    const cx = (r) => r.left + r.width / 2;
    const cy = (r) => r.top + r.height / 2;
    order.forEach((orig, pos) => {
      offs[orig] = { x: cx(a.rects[pos]) - cx(a.rects[orig]), y: cy(a.rects[pos]) - cy(a.rects[orig]) };
    });
  } else {
    let y = a.rects[0].top;
    for (const orig of order) {
      offs[orig] = { x: 0, y: y - a.rects[orig].top };
      y += a.rects[orig].height + a.gap;
    }
  }
  return offs;
}

function layoutOthers(a) {
  const offs = targetOffsets(a);
  a.items.forEach((it, i) => {
    if (i === a.from) return;
    const { x, y } = offs[i];
    it.style.transform = x || y ? `translate(${x / a.scale}px, ${y / a.scale}px)` : '';
  });
}

function onUp(e) {
  if (!active) return;
  e.preventDefault();
  e.stopPropagation();
  const a = active;
  cancelAnimationFrame(a.raf);
  a.raf = 0;

  // Asienta el arrastrado en su hueco antes de cambiar los datos. El
  // desplazamiento es respecto a su sitio original dentro del contenido, así
  // que el scroll que haya habido por medio no cuenta: lo aplica el
  // contenedor a todos por igual.
  const { x, y } = targetOffsets(a)[a.from];
  a.el.classList.add('is-dropping');
  a.el.style.transform = `translate(${x / a.scale}px, ${y / a.scale}px)`;

  // El clic que sigue al pointerup caería en la tecla y la seleccionaría.
  const swallow = (ev) => { ev.stopPropagation(); ev.preventDefault(); };
  window.addEventListener('click', swallow, { capture: true, once: true });
  setTimeout(() => window.removeEventListener('click', swallow, true), 0);

  const { from, to, onDrop, onEnd, items } = a;
  finish({ keepTransforms: true });
  setTimeout(() => {
    a.container.classList.remove('is-sorting');
    if (from !== to) onDrop(from, to);
    // Si onDrop no repinta (o no ha cambiado nada), se deshacen los
    // desplazamientos para que todo vuelva a su sitio.
    for (const it of items) {
      if (!document.contains(it)) continue;
      it.classList.remove('is-dropping', 'is-shifting');
      it.style.transform = '';
    }
    onEnd?.();
  }, DROP_MS);
}

function finish({ keepTransforms = false } = {}) {
  const a = active;
  if (!a) return;
  active = null;
  cancelAnimationFrame(a.raf);
  a.observer?.disconnect();
  window.removeEventListener('pointermove', onMove, true);
  window.removeEventListener('pointerup', onUp, true);
  window.removeEventListener('pointercancel', onCancelEvent, true);
  window.removeEventListener('keydown', onKey, true);
  if (!keepTransforms) a.container.classList.remove('is-sorting');
  a.el.classList.remove('is-dragging');
  document.body.classList.remove('is-sort-dragging');
  if (!keepTransforms) {
    for (const it of a.items) it.classList.remove('is-shifting', 'is-dropping');
  }
}
