"""Regenera OrbyGUI/tips-video/manifest.json a partir de los .mp4 de esa carpeta.

Uso:  python tools/consejos/manifiesto.py [carpeta]

El id de cada vídeo es el nombre del fichero sin extensión y tiene que ser el id de un
consejo de OrbyGUI/src/tips.mjs. La app baja lo que el manifiesto lista y comprueba el
sha256 de cada fichero, así que hay que volver a ejecutarlo cada vez que cambie un vídeo.
"""
import hashlib
import json
import sys
from pathlib import Path

LIMITE = 300 * 1024  # los vídeos se ven pequeños: más peso solo agrandaría la descarga


def main() -> int:
    carpeta = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / "OrbyGUI" / "tips-video"
    videos = []
    grandes = []
    for f in sorted(carpeta.glob("*.mp4")):
        datos = f.read_bytes()
        if len(datos) > LIMITE:
            grandes.append(f"{f.name}: {len(datos) // 1024} KB")
        videos.append({"id": f.stem, "fichero": f.name, "bytes": len(datos), "sha256": hashlib.sha256(datos).hexdigest()})
    if grandes:
        print(f"Pasan de {LIMITE // 1024} KB, comprímelos antes:\n  " + "\n  ".join(grandes), file=sys.stderr)
        return 1
    (carpeta / "manifest.json").write_text(json.dumps({"schema": 1, "videos": videos}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"{len(videos)} vídeo(s) en {carpeta / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
