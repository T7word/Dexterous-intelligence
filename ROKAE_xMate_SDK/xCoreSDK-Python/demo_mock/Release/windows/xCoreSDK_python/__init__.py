"""Mock package ``Release.windows.xCoreSDK_python``.

We re-export the top-level ``xCoreSDK_python`` (in demo_mock/) here, so
that the official ``import setup_path`` -> ``from Release.windows import
xCoreSDK_python`` works exactly like against the real SDK.

Submodules (e.g. ``EventInfoKey``) live in their own directories.
"""

# Re-export everything from the top-level mock as if it were here.
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))
_MOCK_ROOT = _os.path.abspath(_os.path.join(_HERE, "..", "..", ".."))
if _MOCK_ROOT not in _sys.path:
    _sys.path.insert(0, _MOCK_ROOT)

import xCoreSDK_python as _top  # noqa: E402

# Make ``Release.windows.xCoreSDK_python`` accessible by the dotted path
# ``Release.windows.xCoreSDK_python.EventInfoKey`` (the latter lives in
# a sub-folder and is a normal sub-package).
from . import EventInfoKey  # noqa: F401,E402

# Mirror every public name on top-level into our package namespace so
# callers can use either form.
for _name in dir(_top):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_top, _name)

__all__ = getattr(_top, "__all__", [n for n in dir(_top) if not n.startswith("_")])