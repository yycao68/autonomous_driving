"""Shim: di_totg was relocated to avoidance_obstacle/sim/ when the two papers
were merged into one self-contained reproducibility tree. This file re-exports
the real module from there so legacy autonomous_driving/benchmarks/ scripts keep
running; sim/ remains the single source of truth. Do not edit here."""
import importlib.util as _u
import sys as _s
from pathlib import Path as _P

_real = _P(__file__).resolve().parents[2] / "avoidance_obstacle" / "sim" / "di_totg.py"
_spec = _u.spec_from_file_location(__name__, _real)
_mod = _u.module_from_spec(_spec)
_s.modules[__name__] = _mod
_spec.loader.exec_module(_mod)
