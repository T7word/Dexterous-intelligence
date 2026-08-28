"""setup_path replacement for offline mock mode.

Drops the bundled ``demo_mock/Release/...`` folder onto ``sys.path`` so
that the official example scripts can ``import setup_path`` and then do::

    from Release.windows import xCoreSDK_python

exactly the same way they do against the real SDK.

Usage::

    # at the top of an official demo file (or via PYTHONPATH):
    import sys
    sys.path.insert(0, r".../demo_mock")
    import setup_path
    # now the rest of the demo runs unchanged
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELEASE = os.path.join(_HERE, "Release")

# Mirror what the real setup_path.py does
sys.path.append(_RELEASE)
sys.path.append(os.path.join(_RELEASE, "windows"))
sys.path.append(os.path.join(_RELEASE, "linux"))
sys.path.append(os.path.join(_RELEASE, "windows", "xCoreSDK_python"))
sys.path.append(os.path.join(_RELEASE, "linux", "xCoreSDK_python"))
sys.path.append(_HERE)  # so plain ``import xCoreSDK_python`` also works

# also drop demo_mock onto path so the underlying module can be found
sys.path.append(_HERE)