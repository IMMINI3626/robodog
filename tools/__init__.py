import inspect
import types

from . import deflib as _deflib
from . import robodog as _robodog

__version__ = "0.1.5"


def _export_deflib_symbols():
    exported = []
    for name, value in _deflib.__dict__.items():
        if name.startswith("_") or isinstance(value, types.ModuleType):
            continue
        globals()[name] = value
        exported.append(name)
    return exported


def _export_robodog_symbols():
    exported = []
    for name, value in _robodog.__dict__.items():
        if name.startswith("_"):
            continue
        if (inspect.isclass(value) or inspect.isfunction(value)) and getattr(value, "__module__", None) == _robodog.__name__:
            globals()[name] = value
            exported.append(name)
    return exported


__all__ = sorted(set(_export_deflib_symbols() + _export_robodog_symbols()))
