"""Escribe el .sig y el latest.json de una versión de la app, junto a su instalador.

Uso:
  python -I latest_json.py <X.Y.Z> "<firma de sign_update>" "<notas de una línea>" <carpeta nsis>

El .sig queda sin salto de línea final y latest.json en UTF-8 sin BOM: el actualizador compara la
firma tal cual. Comprueba antes que el instalador existe en la carpeta.
"""
import datetime
import json
import os
import sys


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    version, firma, notas, carpeta = sys.argv[1:]
    firma = firma.strip()
    exe = f"Orvy_{version}_x64-setup.exe"
    if not os.path.isfile(os.path.join(carpeta, exe)):
        sys.exit(f"No está el instalador: {os.path.join(carpeta, exe)}")

    with open(os.path.join(carpeta, exe + ".sig"), "w", encoding="utf-8", newline="") as f:
        f.write(firma)

    datos = {
        "version": version,
        "notes": notas,
        "pub_date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "platforms": {"windows-x86_64": {
            "signature": firma,
            "url": f"https://github.com/jaestefaniah27/scroll_wheel/releases/download/v{version}/{exe}",
        }},
    }
    with open(os.path.join(carpeta, "latest.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(datos, f, ensure_ascii=False, indent=2)
    print("escritos", exe + ".sig", "y latest.json en", carpeta)


if __name__ == "__main__":
    main()
