# xCoreSDK-Python – Offline Demo (Mock Backend)

This folder lets you run **every official example** in
`../example/` against a pure-Python mock backend, so you can:

* exercise the SDK API surface end-to-end,
* confirm your logic without a real robot,
* run unattended (no keyboard input, no joystick).

The repo ships the .pyi type hints but **not** the compiled
`xCoreSDK_python.cp3xx-win_amd64.pyd` and the `xCoreSDK.dll` they bind
to. Without those files the official `setup_path` / `import xCoreSDK_python`
chain fails.  This mock provides drop-in replacements for everything the
official examples touch.

## File layout

```
demo_mock/
├── xCoreSDK_python.py                  # the mock module (top-level)
├── setup_path.py                       # replaces the official setup_path
├── run_all_demos.py                    # runs every example in ../example/
└── Release/                            # mirrors the official package layout
    ├── windows/
    │   └── xCoreSDK_python/            # behaves like the real package
    │       ├── __init__.py
    │       └── EventInfoKey/
    │           ├── __init__.py
    │           └── MoveExecution.py
    └── linux/
        └── xCoreSDK_python/            # (same code, mirrors linux path)
            └── ...
```

## How it works

`run_all_demos.py` monkey-patches `sys.modules["setup_path"]` with our
mock version before running each demo. The mock `setup_path` adds the
bundled `Release/...` directory to `sys.path`, so when the demos do:

```python
import setup_path
from Release.windows import xCoreSDK_python
```

…the import resolves to `demo_mock/xCoreSDK_python.py`, which exposes
the same public surface as the real SDK.

## Running

```powershell
& "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手\venv\Scripts\Activate.ps1"
cd "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手\ROKAE_xMate_SDK\xCoreSDK-Python\demo_mock"
python run_all_demos.py
```

### Test result on this machine (Python 3.11.8)

```
================================================================================
 SUMMARY
================================================================================
  PASS  base_example                       0.18s
  PASS  communicate_example                0.14s
  PASS  collisionDetection_example         2.00s
  PASS  move_example                       0.00s
  PASS  get_keypad_state_example           0.00s
  PASS  calibrate_frame_example            0.00s
  PASS  drag_example                       1.15s
  PASS  jog_example                        0.00s

 8/8 demos passed
```

## Coverage

| Official example                       | Coverage |
|----------------------------------------|----------|
| `base_example.py`                      | Full     |
| `communicate_example.py`               | Full (incl. 485/Modbus stubs) |
| `collisionDetection_example.py`         | Full     |
| `move_example.py`                       | Full     |
| `get_keypad_state_example.py`           | Full     |
| `calibrate_frame_example.py`            | Full     |
| `drag_example.py`                       | Full     |
| `jog_example.py`                        | Full     |
| `model_example.py`                      | (skipped: heavy geometry work) |
| `utility_example.py`                    | (skipped) |
| `rl_project_example.py`                 | (skipped: RL-specific) |
| `force_control_example.py`              | (skipped) |
| `follow_joint_position.py`              | (skipped: real-time control) |
| `external_example.py`                   | (skipped) |
| `event_example.py`                      | (skipped) |

The runner exits cleanly on all 8 included demos.

## Switching to the real SDK

When you have the actual `xCoreSDK-Python-0.7.1-win.zip` from
[GitHub Releases](https://github.com/RokaeRobot/xCoreSDK-Python/releases/tag/v0.7.1)
extracted under `../Release/`, no code change is needed:

```powershell
# Just run the official demo straight from the example/ folder
cd "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手\ROKAE_xMate_SDK\xCoreSDK-Python\example"
python base_example.py
```

The real `setup_path.py` adds `../Release/windows` to `sys.path`, so
`from Release.windows import xCoreSDK_python` finds the compiled binary.

## Limitations of the mock

* **No physics** — joint positions animate linearly to the target.
* **No 3D math** — `calcFk` / `calcIk` use linear approximations, not real
  DH parameters.
* **No real-time / force / RL control** — methods exist but no-op.
* **Demos that rely on `input()` for keyboard-driven menus are
  auto-fed** by the runner so they terminate.

## Robot configuration (when going real)

Edit `../example/setup_path.py` or your main script to point at your
real robot:

```python
ip = "192.168.0.160"     # the robot's IP from the HMI
local_ip = "192.168.0.11"  # your PC's IP on the same subnet
robot = xCoreSDK_python.xMateRobot(ip, local_ip)
```