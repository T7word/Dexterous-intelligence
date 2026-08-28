"""Mock package ``Release.linux.xCoreSDK_python``."""
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))
_MOCK_ROOT = _os.path.abspath(_os.path.join(_HERE, "..", "..", ".."))
if _MOCK_ROOT not in _sys.path:
    _sys.path.insert(0, _MOCK_ROOT)

import xCoreSDK_python as _top  # noqa: E402

from . import EventInfoKey  # noqa: F401,E402

for _name in dir(_top):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_top, _name)

__all__ = getattr(_top, "__all__", [n for n in dir(_top) if not n.startswith("_")])