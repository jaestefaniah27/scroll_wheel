// Arranque de la versión de navegador.
//
// El orden importa: window.orby y la plataforma tienen que estar puestos ANTES de
// que se cargue main.js, porque sus módulos consultan las dos cosas nada más
// importarse. Por eso main.js entra por importación dinámica.

import { setPlatform } from './platform.js';
import { instalarOrbyWeb } from './web/orby-web.js';
import { montarGate } from './web/connect-gate.js';
import * as serie from './web/transport-serial.js';

setPlatform('web');
await instalarOrbyWeb();

await import('./main.js');

// El gate se monta después de main.js: main.js pinta la pantalla de carga y engancha
// los eventos del teclado, y el gate tiene que quedar por encima de todo eso.
montarGate();

// Un puerto autorizado en una visita anterior se abre sin pedir nada. Si no hay
// ninguno, esto no hace nada y se queda el botón del gate.
serie.arrancar();
