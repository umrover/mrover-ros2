#!/usr/bin/env python3

import sys
import math
import time
import shutil
import argparse
from pathlib import Path
from typing import Generator
from concurrent.futures import ThreadPoolExecutor, as_completed
from urllib.request import Request, urlopen
from urllib.error import URLError

"""
the mt in the url specifies what sort of layer you want, in our case, it specifies satellite + roads
this url endpoint is not publicly documented
{s} specifies the server to pull from, like mt0 or mt1 to avoid being rate limited
{x}, {y} is coordinates, {z} is zoom
"""

TILE_URL = "https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
CONCURRENCY = 64
OUTPUT = Path(__file__).resolve().parent / "basestation_gui" / "frontend" / "public" / "map"

LOCATIONS = {
    "a2": (42.293988, -83.710156),
    "urc": (38.406422, -110.791900),
    "circ": (51.470181, -112.744886),
}

TIERS = [
    (0, 12, 0.8),
    (13, 16, 0.05),
    (17, 19, 0.015),
    (20, 22, 0.005),
]


def to_tile(lat: float, lon: float, z: int) -> tuple[int, int]:
    n = 2 ** z
    x = int((lon + 180) / 360 * n)
    y = int((1 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2 * n)
    return x, y


def tile_range(lat: float, lon: float, r: float, z: int) -> tuple[int, int, int, int]:
    x0, y1 = to_tile(lat - r, lon - r, z)
    x1, y0 = to_tile(lat + r, lon + r, z)
    m = 2 ** z - 1
    return max(0, x0), max(0, y0), min(m, x1), min(m, y1)


def tiles(lat: float, lon: float) -> Generator[tuple[int, int, int], None, None]:
    for z0, z1, r in TIERS:
        for z in range(z0, z1 + 1):
            x0, y0, x1, y1 = tile_range(lat, lon, r, z)
            for x in range(x0, x1 + 1):
                for y in range(y0, y1 + 1):
                    yield z, x, y


def count(lat: float, lon: float) -> int:
    n = 0
    for z0, z1, r in TIERS:
        for z in range(z0, z1 + 1):
            x0, y0, x1, y1 = tile_range(lat, lon, r, z)
            n += (x1 - x0 + 1) * (y1 - y0 + 1)
    return n


def progress_bar(p: dict) -> None:
    cols = shutil.get_terminal_size().columns
    done, tot = p["done"], p["total"]
    pct = done / tot if tot else 0
    elapsed = time.monotonic() - p["t0"]
    rate = done / elapsed if elapsed else 0
    rem = (tot - done) / rate if rate else 0
    m, s = divmod(int(rem), 60)
    h, m = divmod(m, 60)
    eta = f"{h}h{m:02d}m" if h else f"{m}m{s:02d}s"
    stats = f" {done:,}/{tot:,} | {pct:.1%} | {rate:.0f}/s | ETA {eta} | new:{p['dl']:,} cached:{p['skip']:,} err:{p['err']}"
    w = max(10, cols - len(stats) - 3)
    filled = int(w * pct)
    sys.stderr.write(f"\r[{'#' * filled}{'-' * (w - filled)}]{stats}")
    sys.stderr.flush()


def download(z: int, x: int, y: int) -> str:
    path = OUTPUT / str(z) / str(x) / f"{y}.png"
    if path.exists():
        return "skip"
    url = TILE_URL.format(s=(x + y) % 4, x=x, y=y, z=z)
    req = Request(url, headers={"User-Agent": "Mozilla/5.0"})
    try:
        with urlopen(req, timeout=15) as resp:
            if resp.status == 200:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(resp.read())
                return "dl"
    except (URLError, TimeoutError, OSError):
        pass
    return "err"


def run(lat: float, lon: float, dry_run: bool) -> None:
    tot = count(lat, lon)
    print(f"Tiles: {tot:,} -> {OUTPUT}")
    if dry_run:
        return

    print(f"\nThis will DELETE everything in:\n  {OUTPUT}\nand re-download {tot:,} tiles from Google Maps.\n")
    confirm = input("Continue? [Y/n] ")
    if confirm.strip().lower() not in ("y", ""):
        print("Aborted.")
        return

    if OUTPUT.exists():
        shutil.rmtree(OUTPUT)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    p = {"done": 0, "dl": 0, "skip": 0, "err": 0, "total": tot, "t0": time.monotonic()}

    with ThreadPoolExecutor(max_workers=CONCURRENCY) as pool:
        futures = {}
        pending = 0
        tile_iter = tiles(lat, lon)
        batch = CONCURRENCY * 4

        def submit_batch() -> None:
            nonlocal pending
            for t in tile_iter:
                futures[pool.submit(download, *t)] = t
                pending += 1
                if pending >= batch:
                    return

        submit_batch()
        while futures:
            done_futures = [f for f in futures if f.done()]
            if not done_futures:
                time.sleep(0.01)
                progress_bar(p)
                continue
            for f in done_futures:
                result = f.result()
                p[result] += 1
                p["done"] += 1
                del futures[f]
                pending -= 1
            submit_batch()
            progress_bar(p)

    sys.stderr.write("\n")
    elapsed = time.monotonic() - p["t0"]
    m, s = divmod(int(elapsed), 60)
    print(f"Done in {m}m{s:02d}s: {p['dl']:,} new, {p['skip']:,} cached, {p['err']} errors")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--location", required=True, choices=LOCATIONS)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    lat, lon = LOCATIONS[args.location]
    run(lat, lon, args.dry_run)
