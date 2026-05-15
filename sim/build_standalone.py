#!/usr/bin/env python3
"""Bundle the simulator into a single self-contained HTML file.

Reads sim/index.html, sim/style.css, sim/firmware.js, and sim/sim.js and
emits sim/standalone.html with the CSS inlined in a <style> block and the
JS inlined in a <script type="module"> block (with the `import` statement
removed because everything is now in one scope).

Run:  python3 sim/build_standalone.py
"""
from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def read(name: str) -> str:
    with open(os.path.join(HERE, name), "r", encoding="utf-8") as f:
        return f.read()


def main() -> int:
    html = read("index.html")
    css = read("style.css")
    firmware = read("firmware.js")
    sim = read("sim.js")

    # firmware.js uses `export` keywords; strip them so we can drop everything
    # into a single module scope.
    firmware_inline = re.sub(r"^export\s+", "", firmware, flags=re.MULTILINE)
    # sim.js imports from './firmware.js'; replace that import with nothing
    # because the symbols are already in scope above.
    sim_inline = re.sub(
        r"^\s*import\s+\{[^}]+\}\s+from\s+['\"]\./firmware\.js['\"];\s*$",
        "// (inlined from firmware.js above)",
        sim,
        flags=re.MULTILINE,
    )

    bundled_js = firmware_inline + "\n\n// === sim.js ===\n\n" + sim_inline

    # Replace the <link> + <script> tags with inline blocks.
    out = html
    out = out.replace(
        '<link rel="stylesheet" href="style.css" />',
        f"<style>\n{css}\n</style>",
    )
    # IMPORTANT: emit a plain <script>, NOT <script type="module">. Module
    # scripts from file:// are blocked by the browser's CORS rules (you'd get
    # "Failed to load module script: Cross origin requests are only supported
    # for HTTP."). The bundle has no `import` statements left, so module
    # semantics aren't needed — plain script runs fine off file://.
    out = out.replace(
        '<script type="module" src="sim.js"></script>',
        f'<script>\n{bundled_js}\n</script>',
    )

    # Update title to mark it as the standalone build.
    out = out.replace(
        "<title>Battery Monitor — Simulator</title>",
        "<title>Battery Monitor — Simulator (standalone)</title>",
    )

    dest = os.path.join(HERE, "standalone.html")
    with open(dest, "w", encoding="utf-8") as f:
        f.write(out)

    size_kb = os.path.getsize(dest) / 1024
    print(f"wrote {dest} ({size_kb:.1f} KiB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
