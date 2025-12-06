import sys
import os

# On Windows, we need to explicitly load the bundled TBB DLLs
if sys.platform == "win32":
    import glob
    import ctypes

    # Look for tbb dlls in the same directory as this __init__.py
    _dll_dir = os.path.dirname(__file__)
    for _dll in glob.glob(os.path.join(_dll_dir, "tbb*.dll")):
        try:
            ctypes.CDLL(_dll)
        except Exception as e:
            print(f"Warning: Failed to load {_dll}: {e}")

from dsf_cpp import __version__, LogLevel, get_log_level, set_log_level, mobility, mdt

from .python.cartography import (
    get_cartography,
    graph_from_gdfs,
    graph_to_gdfs,
    create_manhattan_cartography,
)
