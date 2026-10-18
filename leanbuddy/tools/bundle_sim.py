"""Bundles the browser simulator into one self-contained HTML file (the WebAssembly
inlined as base64), for opening without a web server or publishing as a single page.

    python3 tools/bundle_sim.py [OUT.html] [--fragment]

--fragment drops the <!doctype>/<html>/<head>/<body> skeleton and the charset and
viewport metas, for hosts that wrap the page in their own skeleton.
"""
import base64
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else ROOT / "build" / "leanbuddy-sim.html"
    html = (ROOT / "sim" / "index.html").read_text()
    wasm = base64.b64encode((ROOT / "sim" / "leanbuddy.wasm").read_bytes()).decode()
    inline = f'<script>window.LB_WASM_B64 = "{wasm}";</script>\n'
    html = html.replace("<script>\n(() => {", inline + "<script>\n(() => {", 1)
    if "--fragment" in sys.argv:
        for pat in (r"<!doctype html>\s*", r"<html[^>]*>\s*", r"</?head>\s*", r"<body>\s*", r"</body>\s*",
                    r"</html>\s*", r'<meta charset="utf-8">\s*', r'<meta name="viewport"[^>]*>\s*'):
            html = re.sub(pat, "", html, flags=re.I)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f"{out} ({len(html) // 1024} KB)")


if __name__ == "__main__":
    main()
