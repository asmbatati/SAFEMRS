#!/usr/bin/env python3
"""
render_mermaid.py — Convert all .mmd files in this directory to PNG/SVG images.

Usage:
    python render_mermaid.py                        # render all → PNG into ../images/
    python render_mermaid.py -o /custom/out/dir     # custom output directory
    python render_mermaid.py -f svg                 # output SVG instead of PNG
    python render_mermaid.py --files 01 03          # render only matching files
    python render_mermaid.py --backend mmdc         # force local mmdc CLI / npx
    python render_mermaid.py --backend kroki        # force kroki.io (POST API)
    python render_mermaid.py --backend mermaidink   # force mermaid.ink (GET API)
    python render_mermaid.py --backend playwright   # force local headless browser
    python render_mermaid.py --list                 # list available .mmd files

Rendering backends (tried in order unless --backend is specified):
    1. mmdc / npx  — @mermaid-js/mermaid-cli
                     Global: npm install -g @mermaid-js/mermaid-cli
                     Via npx: automatically downloaded on first use (requires npm/npx)
    2. kroki       — https://kroki.io REST API (POST, no install, needs internet)
    3. mermaidink  — https://mermaid.ink (GET, no install, needs internet)
    4. playwright  — Local headless Chromium (fully offline after browser install)
                     Install: pip install playwright && playwright install chromium

Output:
    Images are saved to ../images/ by default (the latex/images/ directory).
    Each file is named <stem>.png (or .svg), e.g. 01_system_architecture.png.
"""

import argparse
import base64
import json
import os
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
import zlib
from pathlib import Path

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

SCRIPT_DIR  = Path(__file__).parent.resolve()
DEFAULT_OUT = SCRIPT_DIR.parent / "images"   # latex/images/
DEFAULT_FMT = "png"
WIDTH       = 1600
BACKGROUND  = "white"

# ---------------------------------------------------------------------------
# Backend 1: mmdc (global) / npx mmdc (on-demand)
# ---------------------------------------------------------------------------

def mmdc_available() -> bool:
    return shutil.which("mmdc") is not None

def npx_available() -> bool:
    return shutil.which("npx") is not None

def render_mmdc(mmd_path: Path, out_path: Path, fmt: str) -> bool:
    """Render via mmdc (global) or npx mmdc (on-demand). Returns True on success."""
    if shutil.which("mmdc"):
        cmd = ["mmdc"]
    elif shutil.which("npx"):
        print("      (using npx mmdc — may download packages on first run)")
        cmd = ["npx", "-y", "@mermaid-js/mermaid-cli"]
    else:
        return False
    try:
        result = subprocess.run(
            cmd + [
                "-i", str(mmd_path),
                "-o", str(out_path),
                "--backgroundColor", BACKGROUND,
                "-w", str(WIDTH),
            ],
            capture_output=True,
            text=True,
            timeout=120,  # npx may download on first run
        )
        if result.returncode == 0:
            return True
        print(f"      mmdc stderr: {result.stderr.strip()[:300]}")
        return False
    except subprocess.TimeoutExpired:
        print("      mmdc timed out (120s)")
        return False
    except Exception as exc:
        print(f"      mmdc exception: {exc}")
        return False

# ---------------------------------------------------------------------------
# Backend 2: kroki.io (POST API — no URL length limit)
# ---------------------------------------------------------------------------

def render_kroki(mmd_text: str, out_path: Path, fmt: str) -> bool:
    """Render via kroki.io REST POST API. Returns True on success."""
    try:
        payload = json.dumps({"diagram_source": mmd_text}).encode("utf-8")
        url     = f"https://kroki.io/mermaid/{fmt}"
        req = urllib.request.Request(
            url,
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Accept": "image/png" if fmt == "png" else "image/svg+xml",
            },
            method="POST",
        )
        print(f"      → POST https://kroki.io/mermaid/{fmt}")
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = resp.read()
        if fmt == "svg":
            out_path.write_text(data.decode("utf-8"), encoding="utf-8")
        else:
            out_path.write_bytes(data)
        return True
    except urllib.error.HTTPError as exc:
        print(f"      kroki.io HTTP {exc.code}: {exc.reason}")
        return False
    except Exception as exc:
        print(f"      kroki.io failed: {exc}")
        return False

# ---------------------------------------------------------------------------
# Backend 3: mermaid.ink (GET with base64 payload)
# ---------------------------------------------------------------------------

def render_mermaidink(mmd_text: str, out_path: Path, fmt: str) -> bool:
    """Render via mermaid.ink (GET with base64-encoded diagram). Returns True on success."""
    try:
        encoded = base64.urlsafe_b64encode(mmd_text.encode("utf-8")).decode("ascii")
        url = f"https://mermaid.ink/img/{encoded}"
        print(f"      → GET  https://mermaid.ink/img/{encoded[:50]}…")
        with urllib.request.urlopen(url, timeout=30) as resp:
            data = resp.read()
        out_path.write_bytes(data)
        return True
    except urllib.error.HTTPError as exc:
        print(f"      mermaid.ink HTTP {exc.code}: {exc.reason}")
        return False
    except Exception as exc:
        print(f"      mermaid.ink failed: {exc}")
        return False

# ---------------------------------------------------------------------------
# Backend 4: Playwright (local headless Chromium)
# ---------------------------------------------------------------------------

def render_playwright(mmd_text: str, out_path: Path, fmt: str) -> bool:
    """
    Render locally using Playwright + Mermaid JS (loaded from CDN).
    Install: pip install playwright && playwright install chromium
    """
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("      playwright not installed.")
        print("      Run: pip install playwright && playwright install chromium")
        return False

    html = f"""<!DOCTYPE html><html><head>
<script src="https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js"></script>
<style>body{{margin:0;background:white;padding:20px}}</style>
</head><body>
<div class="mermaid">
{mmd_text}
</div>
<script>mermaid.initialize({{startOnLoad:true,theme:'default'}});</script>
</body></html>"""

    tmp_html = out_path.parent / f"_tmp_{out_path.stem}.html"
    tmp_html.write_text(html, encoding="utf-8")

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            page = browser.new_page(viewport={"width": 1800, "height": 900})
            page.goto(f"file://{tmp_html}", wait_until="networkidle")
            page.wait_for_timeout(2000)  # let Mermaid finish rendering
            svg_el = page.query_selector("svg")
            if svg_el is None:
                print("      playwright: Mermaid SVG not found in page")
                browser.close()
                return False
            if fmt == "svg":
                content = page.inner_html("svg")
                out_path.write_text(content, encoding="utf-8")
            else:
                svg_el.screenshot(path=str(out_path))
            browser.close()
        return True
    except Exception as exc:
        print(f"      playwright error: {exc}")
        return False
    finally:
        if tmp_html.exists():
            tmp_html.unlink()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def strip_fences(text: str) -> str:
    """Remove optional ```mermaid / ``` code fences."""
    return "\n".join(
        line for line in text.splitlines()
        if not line.strip().startswith("```")
    ).strip()


def find_mmd_files(directory: Path, filters: list) -> list:
    """
    Return sorted list of .mmd files.
    If filters is non-empty, only include files whose name contains any filter string.
    """
    files = sorted(directory.glob("*.mmd"))
    if filters:
        files = [f for f in files if any(flt in f.name for flt in filters)]
    return files


def render_file(mmd_path: Path, out_dir: Path, fmt: str, backend) -> bool:
    """
    Render one .mmd file to <out_dir>/<stem>.<fmt>.
    backend: 'mmdc' | 'kroki' | 'mermaidink' | 'playwright' | None
    None = auto cascade: mmdc/npx → kroki → mermaid.ink → playwright
    """
    out_path = out_dir / f"{mmd_path.stem}.{fmt}"
    raw   = mmd_path.read_text(encoding="utf-8")
    clean = strip_fences(raw)

    # Temp clean file for mmdc (avoids fence-parsing issues)
    tmp = mmd_path.with_suffix(".tmp.mmd")
    tmp.write_text(clean, encoding="utf-8")

    ok = False
    try:
        if backend == "mmdc":
            ok = render_mmdc(tmp, out_path, fmt)
            if not ok:
                print("      ✗ mmdc/npx failed")
        elif backend == "kroki":
            ok = render_kroki(clean, out_path, fmt)
        elif backend == "mermaidink":
            ok = render_mermaidink(clean, out_path, fmt)
        elif backend == "playwright":
            ok = render_playwright(clean, out_path, fmt)
        else:
            # Auto cascade
            if mmdc_available() or npx_available():
                print("      Trying mmdc/npx …")
                ok = render_mmdc(tmp, out_path, fmt)
            if not ok:
                print("      Trying kroki.io …")
                ok = render_kroki(clean, out_path, fmt)
            if not ok:
                print("      Trying mermaid.ink …")
                ok = render_mermaidink(clean, out_path, fmt)
            if not ok:
                print("      Trying playwright …")
                ok = render_playwright(clean, out_path, fmt)
    finally:
        if tmp.exists():
            tmp.unlink()

    return ok

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-o", "--output",
        default=str(DEFAULT_OUT),
        metavar="DIR",
        help=f"Output directory (default: {DEFAULT_OUT})",
    )
    parser.add_argument(
        "-f", "--format",
        choices=["png", "svg"],
        default=DEFAULT_FMT,
        help=f"Output format (default: {DEFAULT_FMT})",
    )
    parser.add_argument(
        "--files",
        nargs="*",
        metavar="PATTERN",
        default=[],
        help="Only render files whose name contains any of these substrings",
    )
    parser.add_argument(
        "--backend",
        choices=["mmdc", "kroki", "mermaidink", "playwright"],
        default=None,
        help="Force a specific backend (default: auto: mmdc/npx → kroki → mermaid.ink → playwright)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available .mmd files and exit",
    )
    args = parser.parse_args()

    out_dir = Path(args.output)
    fmt     = args.format
    backend = args.backend

    # List mode
    if args.list:
        files = find_mmd_files(SCRIPT_DIR, [])
        print("Available .mmd files:")
        for f in files:
            print(f"  {f.name}  ({f.stat().st_size} bytes)")
        return

    mmd_files = find_mmd_files(SCRIPT_DIR, args.files or [])
    if not mmd_files:
        msg = f"No .mmd files matching {args.files}" if args.files else "No .mmd files found"
        print(f"{msg} in {SCRIPT_DIR}")
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)

    # Header
    has_mmdc = mmdc_available()
    has_npx  = npx_available()
    print("=" * 62)
    print("SAFEMRS — Mermaid → Image renderer")
    print(f"  Source dir : {SCRIPT_DIR}")
    print(f"  Output dir : {out_dir}")
    print(f"  Format     : {fmt.upper()}")
    print(f"  Backend    : {backend or 'auto (mmdc/npx → kroki → mermaid.ink → playwright)'}")
    print(f"  Files      : {len(mmd_files)}")
    print(f"  mmdc       : {'✓ found' if has_mmdc else '✗ not found'}")
    print(f"  npx        : {'✓ found' if has_npx  else '✗ not found'}")
    print("=" * 62)

    # Render
    results = {}
    for mmd_path in mmd_files:
        out_path = out_dir / f"{mmd_path.stem}.{fmt}"
        print(f"\n  [RENDER]  {mmd_path.name}  →  {out_path.name}")
        ok = render_file(mmd_path, out_dir, fmt, backend)
        results[mmd_path.name] = ok
        if ok:
            size_kb = out_path.stat().st_size / 1024
            print(f"            ✓  Saved ({size_kb:.1f} KB)")
        else:
            print(f"            ✗  FAILED")

    # Summary
    n_ok  = sum(results.values())
    n_all = len(results)
    print("\n" + "=" * 62)
    print(f"  Done: {n_ok}/{n_all} rendered successfully")
    if n_ok < n_all:
        failed = [n for n, ok in results.items() if not ok]
        print(f"  Failed: {', '.join(failed)}")
        print()
        print("  Quickfix options:")
        print("    npm install -g @mermaid-js/mermaid-cli   # global mmdc")
        print("    python render_mermaid.py --backend mermaidink   # cloud (needs internet)")
        print("    pip install playwright && playwright install chromium  # offline")
    print("=" * 62)
    if n_ok:
        print(f"\n  Images saved to: {out_dir}")

    sys.exit(0 if n_ok == n_all else 1)


if __name__ == "__main__":
    main()
