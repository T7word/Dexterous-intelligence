# ROKAE xMate SDK – Offline Demo (Mock Backend)

This folder contains three demos that exercise the **public API** of the
ROKAE xCore SDK (v0.1.6) without needing the cp38-only `.pyd` binaries
and without needing a real robot.

## Why a mock?

The official Windows pre-compiled SDK ships `.pyd` files built for
**Python 3.8 only**:

```
precompiled_v0.1.6/rokae_SDK_win_v0.1.6_py38/lib/*.cp38-win_amd64.pyd
```

This machine has Python **3.7**, **3.10** and **3.11** available, but
**no 3.8**, so the `.pyd` files cannot be loaded.  Rather than install
another interpreter, `rokae_mock.py` reproduces every method used by
the official `example/*.py` scripts (return types, `ec` dict
semantics, `OperationState` / `OperateMode` / `DragParameter`
enums, …).

When the official SDK is available on a Python 3.8 interpreter, you
can switch backends with one environment variable:

```powershell
$env:ROKAE_SDK_MODE = "real"
python run_demo.py
```

…and the same scripts will use the real SDK instead.

## File layout

```
demo_mock/
├── rokae_mock.py              # pure-Python mock of the ROKAE SDK
├── demo_rokae_firstexample.py # mirrors example/firstexample.py
├── demo_rokae_drag.py         # mirrors example/drag_example.py
├── demo_rokae_threading.py    # mirrors example/threading_example.py
├── run_demo.py                # runs all three, prints PASS/FAIL
└── README.md                  # this file
```

## Running

```powershell
& "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手\venv\Scripts\Activate.ps1"
cd "C:\Users\sxy18\Desktop\记录留痕\20260810三指灵巧手\ROKAE_xMate_SDK\demo_mock"
python run_demo.py
```

Expected output (truncated):

```
========================================================================
 RUNNING: demo_rokae_firstexample
========================================================================
[demo] SDK backend = MOCK
[mock] connectToRobot(127.0.0.1) OK
...
  PASS  demo_rokae_firstexample   6.01s
  PASS  demo_rokae_drag           1.52s
  PASS  demo_rokae_threading      9.12s
 3/3 demos passed
```

## Switching to the real SDK

1. Install Python 3.8 (or use an existing interpreter where the
   official `.pyd` files load).
2. Set `$env:ROKAE_SDK_MODE = "real"` before running.
3. Update `env/robot_config.py` with the IP of your real robot.
4. Make sure the robot controller is on the same LAN and that the
   remote/local IP pair matches the controller's whitelist.

## What the mock does and doesn't do

✅ Implemented (parity with official `example/*.py`):
- `connectToRobot / disconnectFromRobot / setPowerState / powerState`
- `setOperateMode / operateMode / operationState`
- `robotInfo / sdkVersion`
- `jointPos / jointVel / jointTorque / flangePos / baseFrame`
- `toolset / setToolset`
- `calcFK / calcIK` (trivial linear mock, not physically accurate)
- `getDO / setDO / getDI`
- `enableDrag / disableDrag`
- `moveReset / executeCommand / moveStart / pause / stop`
- `getPointPos / adjustSpeedOnline`
- `startRecordPath / saveRecordPath / queryPathLists / replayPath`
- `OperationState / OperateMode / DragParameter` enums
- Background motion thread that animates `joint_pos`
- `MoveLCommand` dataclass

❌ Not implemented (out of scope for the demos):
- Real-time control (`motion_control_rt.h`)
- Force-control (`force_control.h`)
- Collision detection
- Path planning (`planner.h`)
- Live trajectory streaming with servoj
- Actual physical motion (obviously)