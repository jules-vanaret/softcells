"""
SoftCells - A soft body physics simulation library.

This package provides a comprehensive soft body physics simulation system
with features including:
- Verlet integration for stable physics
- Spring-mass systems
- Pressure-based volume preservation
- Collision detection and resolution
- Decoupled physics engine and visualization

Main Components:
- core: Basic physics components (PointMass, Spring)
- shapes: Shape implementations (Shape, CircleShape)
- simulation: Physics engine and collision handling
- visualization: Pygame-based visualization layer
- utils: Utility functions for geometry and math
- config: Configuration constants and settings
"""

from .core import PointMass, Spring
from .shapes import Shape, CircleShape
from .simulation import PhysicsEngine, CollisionHandler
from .visualization import SimulationVisualizer
from .config import *

__version__ = "1.0.0"
__author__ = "SoftCells Development Team"

__all__ = [
    'PointMass', 'Spring', 'Shape', 'CircleShape', 
    'PhysicsEngine', 'CollisionHandler',
    'SimulationVisualizer'
] 