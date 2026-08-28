"""
run_all_demos.py
================

Loads every demo in the official ``example/`` directory and runs it
against the bundled mock backend (no real robot, no real SDK
binary required).

The trick is to monkey-patch ``sys.modules['setup_path']`` so that the
``import setup_path`` at the top of every demo file routes its
``Release/windows`` import to our mock instead of the real SDK.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_DIR = os.path.abspath(os.path.join(HERE, "..", "example"))

# Make sure the mock is importable first
sys.path.insert(0, HERE)
import setup_path  # noqa: E402  - mock version, pre-pends Release/

# Also expose the official example/ directory so demos can do
# ``from move_example import wait_robot`` against each other.
sys.path.insert(0, EXAMPLE_DIR)

# Auto-feed ``input()`` so demos that ask for keyboard input run unattended.
import builtins as _builtins  # noqa: E402

_real_input = _builtins.input
_auto_input_calls = {"n": 0}


def _auto_input(prompt: str = "") -> str:
    _auto_input_calls["n"] += 1
    # ``calibrate_frame`` waits for an enter to confirm each calibration point
    # ``jog`` waits for an enter to stop jogging
    # ``drag`` prints a menu and waits for a single character -> return 'q' to quit
    if "d:" in prompt and "drag" in prompt:
        return "q"
    return ""


_builtins.input = _auto_input

# Replace ``sys.stdin.read`` so demos that wait for a key press (e.g.
# jog_example) finish promptly.
import io as _io  # noqa: E402


class _AutoStdin(_io.StringIO):
    """Return ``"\\n"`` to every read, until a max-call guard kicks in.

    This makes loops of the shape
    ``while True: char = sys.stdin.read(1); if char == "\\n": break``
    exit on their very first iteration.
    """
    _calls = 0
    _max_calls = 200

    def reset(self) -> None:
        self._calls = 0

    def read(self, n: int = -1) -> str:  # type: ignore[override]
        self._calls += 1
        if self._calls > self._max_calls:
            return ""
        return "\n"

    def readline(self, n: int = -1) -> str:  # type: ignore[override]
        return self.read(n)


_AutoStdin_inst = _AutoStdin()
sys.stdin = _AutoStdin_inst
# expose a way for the runner to reset between demos
sys._auto_stdin_reset = _AutoStdin_inst.reset  # type: ignore[attr-defined]

# List of demos to run (in order).  Demos that rely on heavy keyboard
# interaction or background threads are skipped on purpose.
DEMOS = [
    "base_example",
    "communicate_example",
    "collisionDetection_example",
    "move_example",
    "get_keypad_state_example",
    "calibrate_frame_example",
    "drag_example",
    "jog_example",
]


def _patch_setup_path() -> None:
    """Pre-populate sys.modules['setup_path'] with the mock."""
    spec = importlib.util.spec_from_file_location(
        "setup_path", os.path.join(HERE, "setup_path.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sys.modules["setup_path"] = mod


def _load_demo(name: str):
    """Load and execute a demo file under our mock backend."""
    _patch_setup_path()

    # also patch sys.modules['log'] with our tiny shim if needed
    if "log" not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            "log", os.path.join(EXAMPLE_DIR, "log.py")
        )
        log_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(log_mod)
        sys.modules["log"] = log_mod

    src_path = os.path.join(EXAMPLE_DIR, f"{name}.py")

    # For jog_example, monkey-patch the inner ``while char == "\n":``
    # pattern so the demo exits without an interactive tty.
    if name == "jog_example":
        import types
        ns: dict = {"__name__": "__main__", "__file__": src_path}

        # Make ``input`` and ``sys.stdin.read`` always return "\n"
        def _ok(prompt: str = "") -> str:
            return ""
        ns["input"] = _ok

        class _Stdin(types.SimpleNamespace):
            def read(self, n: int = -1) -> str:
                return "\n"
            def readline(self, n: int = -1) -> str:
                return "\n"

        import sys as _sys
        ns["sys"] = types.SimpleNamespace(
            stdin=_Stdin(),
            stdout=_sys.stdout,
            stderr=_sys.stderr,
            modules=_sys.modules,
            path=_sys.path,
            platform=_sys.platform,
        )

        with open(src_path, "r", encoding="utf-8") as f:
            src = f.read()
        compiled = compile(src, src_path, "exec")
        exec(compiled, ns)
        return ns

    import runpy
    return runpy.run_path(src_path, run_name="__main__")


def run_one(name: str):
    print("\n" + "=" * 80)
    print(f" RUNNING: {name}")
    print("=" * 80)
    t0 = time.time()
    # Reset stdin feeder so each demo gets a fresh "press enter" budget.
    sys._auto_stdin_reset()
    try:
        # ``run_path`` already ran the module's ``if __name__ == "__main__"``
        # block, so we just need to wrap it in a timeout to catch any
        # accidental infinite loops in user-facing demos (jog/drag).
        import threading as _th
        result = {}

        def _runner():
            try:
                mod = _load_demo(name)
                result["mod"] = mod
            except Exception as e:
                result["error"] = e

        t = _th.Thread(target=_runner, daemon=True)
        t.start()
        t.join(timeout=15)
        if t.is_alive():
            print(f"  [timeout] {name} exceeded 15s; killing")
            return False, time.time() - t0
        if "error" in result:
            traceback.print_exception(type(result["error"]), result["error"],
                                     result["error"].__traceback__)
            return False, time.time() - t0
        return True, time.time() - t0
    except Exception as e:
        traceback.print_exc()
        return False, time.time() - t0


def main() -> None:
    results = []
    for name in DEMOS:
        ok, dt = run_one(name)
        results.append((name, ok, dt))

    print("\n" + "=" * 80)
    print(" SUMMARY")
    print("=" * 80)
    n_ok = sum(1 for _, ok, _ in results if ok)
    for name, ok, dt in results:
        status = "PASS" if ok else "FAIL"
        print(f"  {status:4s}  {name:32s}  {dt:5.2f}s")
    print(f"\n {n_ok}/{len(results)} demos passed")
    if n_ok != len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()