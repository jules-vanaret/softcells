"""
Utility modules for the soft body simulation.
"""

from .geometry import orientation, on_segment, vectorized_orientations, pbc_operator, winding_vector

__all__ = ['orientation', 'on_segment', 'vectorized_orientations', 'pbc_operator', 'winding_vector'] 