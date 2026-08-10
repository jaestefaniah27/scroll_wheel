// Copias de seguridad sin diálogo nativo.
//
// En el escritorio esto lo hacen dialog.showSaveDialog / showOpenDialog del proceso
// principal. En el navegador se resuelve con una descarga y un <input type="file">,
// que funcionan en todos los navegadores sin pedir permisos.
//
// La forma de lo que devuelven tiene que ser la misma que la del proceso principal
// ({ ok, canceled, data, error }): src/backup.js no debe enterarse de dónde corre.

function nombreArchivo() {
  const d = new Date();
  const dos = (n) => String(n).padStart(2, '0');
  return `orby-backup-${d.getFullYear()}-${dos(d.getMonth() + 1)}-${dos(d.getDate())}`
       + `-${dos(d.getHours())}-${dos(d.getMinutes())}.json`;
}

export async function saveBackup(data) {
  try {
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = nombreArchivo();
    a.click();
    // Sin esto el blob se queda en memoria toda la sesión, y una copia con todos
    // los iconos no es precisamente pequeña.
    setTimeout(() => URL.revokeObjectURL(url), 10_000);
    return { ok: true };
  } catch (err) {
    return { ok: false, error: err.message };
  }
}

export function loadBackup() {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'application/json,.json';

    input.addEventListener('change', async () => {
      const file = input.files?.[0];
      if (!file) { resolve({ ok: false, canceled: true }); return; }
      try {
        resolve({ ok: true, data: JSON.parse(await file.text()) });
      } catch (err) {
        resolve({ ok: false, error: `El archivo no es un JSON válido: ${err.message}` });
      }
    });

    // 'cancel' no lo emiten todos los navegadores; cuando no llega, la promesa se
    // queda colgada y el botón de restaurar se queda en "Restaurando…" para
    // siempre. El foco volviendo a la ventana es la señal que sí llega en todos.
    window.addEventListener('focus', () => {
      setTimeout(() => {
        if (!input.files?.length) resolve({ ok: false, canceled: true });
      }, 500);
    }, { once: true });

    input.click();
  });
}
