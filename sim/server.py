#!/usr/bin/env python3
"""Tiny dev server for the Battery Monitor simulator.

Run from this directory:
    python3 server.py

Then open http://localhost:8765 in any modern browser.

The simulator is a pure static-asset app (HTML + CSS + ES modules), so any
static file server works — this script is just a one-command convenience.
"""
from __future__ import annotations

import http.server
import os
import socketserver
import sys
import webbrowser
from functools import partial

DEFAULT_PORT = 8765


def main(argv: list[str]) -> int:
    port = int(argv[1]) if len(argv) > 1 else DEFAULT_PORT
    here = os.path.dirname(os.path.abspath(__file__))
    handler = partial(http.server.SimpleHTTPRequestHandler, directory=here)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", port), handler) as srv:
        url = f"http://localhost:{port}/"
        print(f"Battery Monitor simulator → {url}")
        print("Ctrl-C to stop.")
        if "--open" in argv:
            webbrowser.open(url)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print()
            return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
