"""Make ``Release.windows`` a package.

The real sub-package ``Release.windows.xCoreSDK_python`` lives in its
own directory (``xCoreSDK_python/__init__.py``), so we must NOT register
``xCoreSDK_python`` here as a module — that would shadow the package.
"""