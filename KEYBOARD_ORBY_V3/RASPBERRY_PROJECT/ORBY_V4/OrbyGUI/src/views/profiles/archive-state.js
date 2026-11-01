// La lista de perfiles archivados que la app tiene en memoria, aparte de
// archive.js para que macros-store.js pueda mirarla sin importar el módulo que
// escribe perfiles (que a su vez importa macros-store.js: un ciclo).
//
// null = todavía no se ha leído del teclado; [] = se leyó y no hay ninguno.
let list = null;

export const archived = () => list;
export const setArchived = (next) => { list = next; };
