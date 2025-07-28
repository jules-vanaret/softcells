"""
Collision detection and resolution for soft body physics.
"""

import math

from ..config import COLLISION_SLOP, COLLISION_CORRECTION_PERCENT, COLLISION_RESTITUTION, PERIODIC, DEFAULT_WIDTH, DEFAULT_HEIGHT
from ..utils import pbc_operator

class CollisionHandler:
    """
    Handles collision detection and resolution between shapes.
    """
    
    def __init__(self):
        """Initialize the collision handler."""
        self.slop = COLLISION_SLOP
        self.correction_percent = COLLISION_CORRECTION_PERCENT
        self.restitution = COLLISION_RESTITUTION
    
    def handle_collisions(self, shapes):
        """
        Detect and resolve collisions between shapes.
        
        Args:
            shapes (list): List of Shape objects to check for collisions
        """
        num_shapes = len(shapes)
        if num_shapes < 2:
            return

        for i in range(num_shapes):
            for j in range(i + 1, num_shapes):
                shape_a = shapes[i]
                shape_b = shapes[j]

                # --- BROAD PHASE: Bounding Box Check ---
                min_ax, max_ax, min_ay, max_ay = shape_a._get_bounding_box()
                min_bx, max_bx, min_by, max_by = shape_b._get_bounding_box()

                # If the bounding boxes do not overlap, skip to the next pair
                if max_ax < min_bx or min_ax > max_bx or max_ay < min_by or min_ay > max_by:
                    continue  # Not colliding, so no need for the expensive check

                # Test points of A inside B
                for point in shape_a.get_points():
                    if shape_b.is_point_inside(point):
                        self.resolve_collision(point, shape_b)

                # Test points of B inside A
                for point in shape_b.get_points():
                    if shape_a.is_point_inside(point):
                        self.resolve_collision(point, shape_a)

    def resolve_collision(self, colliding_point, shape):
        """
        Resolve overlap and bounce between a point‑mass and a polygon edge,
        now aware of rectangular PERIODIC boundaries (PERIODIC flag).

        Args
        ----
        colliding_point : PointMass
        shape           : Shape (provides find_closest_edge)
        """
        edge_p1, edge_p2, closest_point_data = shape.find_closest_edge(colliding_point)
        if edge_p1 is None:                      # no edge found
            return

        cx, cy, t = closest_point_data          # closest point on edge   (0 ≤ t ≤ 1)

        # --- 1. POSITION RESOLUTION ----------------------------------------
        dx = pbc_operator(colliding_point.x - cx, DEFAULT_WIDTH)
        dy = pbc_operator(colliding_point.y - cy, DEFAULT_HEIGHT)
        penetration = math.hypot(dx, dy)
        if penetration < 1e-6:
            return                                # already outside / on the edge

        # inward normal (edge → point)
        nx, ny = dx / penetration, dy / penetration

        corr_depth = max(penetration - self.slop, 0.0)
        if corr_depth == 0.0:
            return

        inv_m_p  = 1.0 / colliding_point.mass if colliding_point.mass > 0 else 0.0
        inv_m_e1 = 1.0 / edge_p1.mass            if edge_p1.mass          > 0 else 0.0
        inv_m_e2 = 1.0 / edge_p2.mass            if edge_p2.mass          > 0 else 0.0
        total_inv_m = inv_m_p + inv_m_e1 * (1 - t) + inv_m_e2 * t
        if total_inv_m < 1e-6:
            return                                # infinite mass system – nothing to move

        move = self.correction_percent * corr_depth / total_inv_m

        def _shift(p, sx, sy):
            """Move point and wrap back into the PERIODIC box if needed."""
            p.x += sx
            p.y += sy
            if PERIODIC: p.x = pbc_operator(p.x, DEFAULT_WIDTH)
            if PERIODIC : p.y = pbc_operator(p.y, DEFAULT_HEIGHT)

        # point moves OUTward (‑normal); edge vertices move INward (+normal)
        _shift(colliding_point, -nx * move * inv_m_p,               -ny * move * inv_m_p)
        _shift(edge_p1,          nx * move * inv_m_e1 * (1 - t),     ny * move * inv_m_e1 * (1 - t))
        _shift(edge_p2,          nx * move * inv_m_e2 * t,           ny * move * inv_m_e2 * t)

        # --- 2. VELOCITY RESOLUTION ----------------------------------------
        edge_vx = edge_p1.vx * (1 - t) + edge_p2.vx * t
        edge_vy = edge_p1.vy * (1 - t) + edge_p2.vy * t

        rel_vx  = colliding_point.vx - edge_vx
        rel_vy  = colliding_point.vy - edge_vy
        vel_n   = rel_vx * nx + rel_vy * ny        # component along normal

        if vel_n > 0.0:                            # already separating
            return

        j = -(1 + self.restitution) * vel_n / total_inv_m   # impulse magnitude

        colliding_point.vx -= j * inv_m_p  * nx
        colliding_point.vy -= j * inv_m_p  * ny

        edge_p1.vx          += j * inv_m_e1 * (1 - t) * nx
        edge_p1.vy          += j * inv_m_e1 * (1 - t) * ny

        edge_p2.vx          += j * inv_m_e2 * t * nx
        edge_p2.vy          += j * inv_m_e2 * t * ny