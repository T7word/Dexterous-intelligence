"""
run_demo.py
===========

Runs every offline demo in this folder in sequence and prints a single
PASS/FAIL summary.  This is the recommended entry point.
"""

from __future__ import annotations

import importlib
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEMOS = [
    "demo_rokae_firstexample",
    "demo_rokae_drag",
    "demo_rokae_threading",
]


def run_one(name: str) -> tuple[bool, float]:
    print("\n" + "=" * 72)
    print(f" RUNNING: {name}")
    print("=" * 72)
    t0 = time.time()
    try:
        mod = importlib.import_module(name)
        mod.main()
        return True, time.time() - t0
    except Exception:
        traceback.print_exc()
        return False, time.time() - t0


def main() -> None:
    results = []
    for name in DEMOS:
        ok, dt = run_one(name)
        results.append((name, ok, dt))

    print("\n" + "=" * 72)
    print(" SUMMARY")
    print("=" * 72)
    n_ok = sum(1 for _, ok, _ in results if ok)
    for name, ok, dt in results:
        status = "PASS" if ok else "FAIL"
        print(f"  {status:4s}  {name:32s}  {dt:5.2f}s")
    print(f"\n {n_ok}/{len(results)} demos passed")

    if n_ok != len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()