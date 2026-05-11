"""fleet_math — Core fleet mathematics for graph consensus and rigidity.

Modules:
  zhc   — Zero Holonomy Consensus (constraint graphs)
  h1    — H1 emergence detection (first Betti number)
  laman — Laman rigidity for 2D bar-joint frameworks
  field — Continuous constraint field interpolation
"""

from .zhc import ConstraintGraph
from .h1 import betti_1, emergence_severity, detect_emergence, connected_components
from .laman import is_rigid, is_minimally_rigid, rigid_margin
from .field import Field

__all__ = [
    "ConstraintGraph",
    "betti_1",
    "emergence_severity",
    "detect_emergence",
    "connected_components",
    "is_rigid",
    "is_minimally_rigid",
    "rigid_margin",
    "Field",
]

__version__ = "0.1.0"
