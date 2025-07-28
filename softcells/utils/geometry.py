"""
Geometry utilities for collision detection and geometric calculations.
Optimized with Numba for performance.
"""

from numba import njit


@njit(cache=True)
def orientation(p, q, r):
    """
    Calculate the orientation of the ordered triplet (p, q, r).
    
    Args:
        p, q, r: Points as (x, y) tuples
    
    Returns:
        int: 0 if collinear, 1 if clockwise, 2 if counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0  # Collinear
    return 1 if val > 0 else 2  # Clockwise or Counterclockwise


@njit(cache=True)
def on_segment(p, q, r):
    """
    Check if point q lies on line segment pr.
    
    Args:
        p, q, r: Points as (x, y) tuples
    
    Returns:
        bool: True if q lies on segment pr
    """
    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))


@njit(cache=True)
def vectorized_orientations(p1, q1, p2, q2):
    """
    Vectorized orientation calculation for performance with numba.
    Used for line segment intersection testing.
    
    Args:
        p1, q1: First line segment endpoints as (x, y) tuples
        p2, q2: Second line segment endpoints as (x, y) tuples
    
    Returns:
        tuple: Four orientation values for intersection testing
    """
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    return o1, o2, o3, o4 

def pbc_operator(x, L, return_wrap=False):
    """
    Apply periodic boundary condition operator.
    
    Args:
        x: Value to apply PBC to
        L: Length of the periodic boundary

    Returns:
        float: Adjusted value within the periodic boundary
    """
    x_new = (x + L/2) % L - L/2
    if return_wrap:
        return x_new, x_new == x
    return x_new