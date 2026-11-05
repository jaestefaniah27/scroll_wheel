"""Crea una release en jaestefaniah27/scroll_wheel y sube sus ficheros.

Uso:
  python -I crear_release.py <etiqueta> "<título>" <notas.md> (--prerelease | --latest) <ficheros...>

  --prerelease  firmware (fw-v*): no cuenta como "latest" y no toca el actualizador de la app.
  --latest      app (v*): queda como la última release, que es la que lee el actualizador.

La etiqueta tiene que existir ya en el remoto. El token sale de `git credential fill` y no se
imprime nunca. Al acabar dice qué release ha quedado como "latest".
"""
import json
import os
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request

REPO = "jaestefaniah27/scroll_wheel"
TIPOS = {".json": "application/json", ".sig": "text/plain", ".sha256": "text/plain"}


def token():
    out = subprocess.run(["git", "credential", "fill"], input="protocol=https\nhost=github.com\n\n",
                         capture_output=True, text=True, check=True).stdout
    campos = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
    if not campos.get("password"):
        sys.exit("No hay credencial de GitHub guardada para git.")
    return campos["password"]


def main():
    args = sys.argv[1:]
    modo = [a for a in args if a in ("--prerelease", "--latest")]
    args = [a for a in args if a not in ("--prerelease", "--latest")]
    if len(modo) != 1 or len(args) < 4:
        sys.exit(__doc__)
    etiqueta, titulo, notas, *ficheros = args
    pre = modo[0] == "--prerelease"
    for f in ficheros:
        if not os.path.isfile(f):
            sys.exit(f"No existe: {f}")

    cabeceras = {"Authorization": "Bearer " + token(), "Accept": "application/vnd.github+json",
                 "X-GitHub-Api-Version": "2022-11-28", "User-Agent": "orvy-publicar"}

    def llamar(metodo, url, datos=None, tipo="application/json"):
        h = dict(cabeceras)
        if datos is not None:
            h["Content-Type"] = tipo
        try:
            with urllib.request.urlopen(urllib.request.Request(url, data=datos, method=metodo, headers=h)) as r:
                cuerpo = r.read()
                return json.loads(cuerpo) if cuerpo else None
        except urllib.error.HTTPError as e:
            sys.exit(f"HTTP {e.code}: {e.read().decode()[:500]}")

    rel = llamar("POST", f"https://api.github.com/repos/{REPO}/releases", json.dumps({
        "tag_name": etiqueta, "name": titulo, "body": open(notas, encoding="utf-8").read(),
        "draft": False, "prerelease": pre, "make_latest": "false" if pre else "true",
    }).encode())
    print("release", rel["html_url"], "prerelease =", rel["prerelease"])

    subida = rel["upload_url"].split("{")[0]
    for f in ficheros:
        nombre = os.path.basename(f)
        tipo = TIPOS.get(os.path.splitext(nombre)[1], "application/octet-stream")
        a = llamar("POST", f"{subida}?name={urllib.parse.quote(nombre)}", open(f, "rb").read(), tipo)
        print("subido", a["name"], a["size"])

    print("latest =", llamar("GET", f"https://api.github.com/repos/{REPO}/releases/latest")["tag_name"])


if __name__ == "__main__":
    main()
