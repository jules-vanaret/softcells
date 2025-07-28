"""
Base shape implementation for soft body physics.
This module contains no rendering dependencies.
"""

import math

from ..core import Spring
from ..utils.geometry import vectorized_orientations, on_segment, pbc_operator
from ..config import (
    DEFAULT_SHAPE_COLOR, DEFAULT_LINE_WIDTH, COLLISION_SLOP, 
    COLLISION_CORRECTION_PERCENT, COLLISION_RESTITUTION,
    PERIODIC, DEFAULT_WIDTH, DEFAULT_HEIGHT,
)


class Shape:
    """
    A shape is a collection of point masses connected in order.
    When you connect the points in order with line segments, you create a shape.
    Supports pressure-based physics to maintain shape integrity.
    """
    
    def __init__(self, points=None):
        """
        Initialize a shape with a list of point masses.
        
        Args:
            points (list): List of PointMass objects
        """
        self.points = points if points is not None else []
        self.color = DEFAULT_SHAPE_COLOR
        self.line_width = DEFAULT_LINE_WIDTH
        
        # Pressure physics parameters
        self.pressure_enabled = True
        self.pressure_amount = 0
        self.normal_vectors = []  # Normal vectors for each edge
        self.edge_lengths = []    # Length of each edge
        self.damping_factor = 0.98  # Velocity damping to reduce oscillations
        self.initial_volume = 0.0   # Target volume for pressure regulation
        self.current_volume = 0.0   # Current volume
        self.scalar_pressure = 0.0  # Calculated pressure value
        
        # Spring physics parameters
        self.springs = []  # List of Spring objects connecting points
        self.spring_stiffness = 100.0  # Default spring stiffness
        self.spring_damping = 10.0     # Default spring damping
        self.springs_enabled = True    # Enable/disable spring forces
        
        # Drag physics parameters
        self.drag_coefficient = 0.0    # Default drag coefficient
        self.drag_type = "linear"      # Default drag type ("linear" or "quadratic")
        self.drag_enabled = True       # Enable/disable drag forces
    
    def add_point(self, point):
        """Add a point mass to this shape."""
        self.points.append(point)
    
    def get_points(self):
        """Get all point masses in this shape."""
        return self.points
    
    def get_positions(self):
        """Get positions of all points as a list of (x, y) tuples."""
        return [(point.x, point.y) for point in self.points]
    
    def apply_force_to_all(self, fx, fy):
        """Apply a force to all points in this shape."""
        for point in self.points:
            point.apply_force(fx, fy)
    
    def update_all(self, dt):
        """Update physics for all points in this shape."""
        for point in self.points:
            point.update(dt)
            # Apply damping to reduce oscillations
            point.vx *= self.damping_factor
            point.vy *= self.damping_factor
    
    def set_color(self, color):
        """Set the rendering color for this shape."""
        self.color = color
    
    def set_pressure(self, pressure_amount):
        """Set the internal gas pressure amount."""
        self.pressure_amount = pressure_amount
    
    def enable_pressure(self, enabled=True):
        """Enable or disable pressure physics."""
        self.pressure_enabled = enabled
    
    def set_spring_properties(self, stiffness, damping):
        """Set spring stiffness and damping for all springs in this shape."""
        self.spring_stiffness = stiffness
        self.spring_damping = damping
        # Update existing springs
        for spring in self.springs:
            spring.stiffness = stiffness
            spring.damping = damping
    
    def enable_springs(self, enabled=True):
        """Enable or disable spring physics."""
        self.springs_enabled = enabled
    
    def set_drag_properties(self, drag_coefficient, drag_type="linear"):
        """Set drag coefficient and type for all points in this shape."""
        self.drag_coefficient = drag_coefficient
        self.drag_type = drag_type
        # Update all points in the shape
        for point in self.points:
            point.set_drag_coefficient(drag_coefficient)
    
    def enable_drag(self, enabled=True):
        """Enable or disable drag physics."""
        self.drag_enabled = enabled
    
    def apply_drag_forces(self):
        """Apply drag forces to all points in this shape."""
        if not self.drag_enabled:
            return
        
        for point in self.points:
            point.apply_drag_force(self.drag_type)
    
    def create_springs(self):
        """Create springs between adjacent points in the shape."""
        if len(self.points) < 2:
            return
        
        self.springs.clear()
        num_points = len(self.points)
        
        # Create springs between adjacent points (including wrapping around)
        for i in range(num_points):
            point1 = self.points[i]
            point2 = self.points[(i + 1) % num_points]  # Wrap around to close the shape
            
            spring = Spring(point1, point2, self.spring_stiffness, self.spring_damping)
            self.springs.append(spring)
    
    def apply_spring_forces(self):
        """Apply spring forces between connected point masses."""
        if not self.springs_enabled:
            return
        
        for spring in self.springs:
            spring.apply_force()
    
    def calculate_volume(self):
        """
        Calculate the area of the closed 2D polygon using Gauss's theorem.
        Also fills self.normal_vectors and self.edge_lengths.
        """
        if len(self.points) < 3:
            return 0.0

        volume = 0.0
        num_points = len(self.points)
        self.normal_vectors = []
        self.edge_lengths = []

        for i in range(num_points):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % num_points]  # wrap around

            dx = p2.x - p1.x
            dy = p2.y - p1.y

            if PERIODIC:
                dx, wrapped_x = pbc_operator(dx, DEFAULT_WIDTH, return_wrap=True)
                dy, wrapped_y = pbc_operator(dy, DEFAULT_HEIGHT, return_wrap=True)

            edge_length = math.hypot(dx, dy)
            self.edge_lengths.append(edge_length)

            # Normal vector (perpendicular, outward orientation depends on CCW order)
            if edge_length > 1e-9:
                nx = dy / edge_length
                ny = -dx / edge_length
            else:
                nx = ny = 0.0

            self.normal_vectors.append((nx, ny))

            # Shoelace contribution: x1*y2 - x2*y1
            dv = (p1.x * p2.y - p2.x * p1.y)
            if PERIODIC:
                volume += dv  * (-1)**(wrapped_x + wrapped_y)
            else:
                volume += dv

        return abs(volume) * 0.5
    
    def apply_pressure_forces(self):
        """
        Apply pressure forces to maintain the shape's volume.
        Based on the pressure model from the soft body paper, but stabilized.
        """
        if not self.pressure_enabled or len(self.points) < 3:
            return
        
        if not self.initial_volume:
            self.initial_volume = self.calculate_volume()

        self.current_volume = self.calculate_volume()

        self.scalar_pressure = self.pressure_amount * self.initial_volume /  self.current_volume 
        # Limit pressure force to prevent instability
        max_pressure = self.scalar_pressure
        self.scalar_pressure = max(-max_pressure, min(max_pressure, self.scalar_pressure))

        num_points = len(self.points)
        
        # Apply pressure force to each edge
        for i in range(num_points):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % num_points]
            
            if i < len(self.normal_vectors) and i < len(self.edge_lengths):
                nx, ny = self.normal_vectors[i]
                edge_length = self.edge_lengths[i]
                
                # Calculate pressure force magnitude (scaled by edge length)
                pressure_magnitude = self.scalar_pressure * edge_length   # |F| = P*A
                
                # Calculate force components
                fx = nx * pressure_magnitude
                fy = ny * pressure_magnitude
                
                # Apply force to both endpoints of the edge
                # This distributes the pressure force evenly
                p1.apply_force(fx * 0.5, fy * 0.5)
                p2.apply_force(fx * 0.5, fy * 0.5)
    
    def _get_bounding_box(self):
        """
        Return (xmin, xmax, ymin, ymax).

        * non‑periodic → usual bbox
        * periodic     → bbox of the *unwrapped* polygon, so the limits may
                        be outside the primitive box.
        """
        if not self.points:
            return 0.0, 0.0, 0.0, 0.0

        if not PERIODIC:
            xs = [p.x for p in self.points]
            ys = [p.y for p in self.points]
        else:
            xs, ys = self._unwrap_polygon(self.points, DEFAULT_WIDTH, DEFAULT_HEIGHT)

        return min(xs), max(xs), min(ys), max(ys)
    
    def _unwrap_polygon(self, points, box_w, box_h):
        """Minimal‑image unwrap -> continuous vertex list (xs, ys)."""
        xs = [points[0].x]
        ys = [points[0].y]

        for p in points[1:]:
            dx = p.x - xs[-1]
            dy = p.y - ys[-1]
            if box_w:                 # apply only if box size > 0
                dx -= round(dx / box_w) * box_w
            if box_h:
                dy -= round(dy / box_h) * box_h
            xs.append(xs[-1] + dx)
            ys.append(ys[-1] + dy)

        return xs, ys

    def is_point_inside(self, test_point):
        """
        Ray‑casting point‑in‑polygon test (even–odd rule) with optional
        rectangular periodic boundaries.

        Args
        ----
        test_point : PointMass

        Returns
        -------
        bool
        """
        if len(self.points) < 3:
            return False

        # -- unwrap polygon so edges do not jump across the box ---------------
        if PERIODIC:
            xs, ys = self._unwrap_polygon(self.points, DEFAULT_WIDTH, DEFAULT_HEIGHT)

            # shift polygon by integer box vectors so it's closest to test_point
            shift_x = round((xs[0] - test_point.x) / DEFAULT_WIDTH) if DEFAULT_WIDTH else 0
            shift_y = round((ys[0] - test_point.y) / DEFAULT_HEIGHT) if DEFAULT_HEIGHT else 0
            xs = [x - shift_x * DEFAULT_WIDTH for x in xs]
            ys = [y - shift_y * DEFAULT_HEIGHT for y in ys]

            poly = list(zip(xs, ys))
        else:
            poly = [(p.x, p.y) for p in self.points]

        # -- classic ray‑casting from test_point horizontally -----------------
        p1 = (test_point.x, test_point.y)
        max_x = max(v[0] for v in poly)
        q1 = (max_x + (DEFAULT_WIDTH if PERIODIC else 10.0) + 1.0, test_point.y)   # point outside

        intersections = 0
        n = len(poly)

        for i in range(n):
            p2 = poly[i]
            q2 = poly[(i + 1) % n]

            o1, o2, o3, o4 = vectorized_orientations(p1, q1, p2, q2)

            # general intersection
            if o1 != o2 and o3 != o4:
                intersections += 1
                continue

            # collinear special cases
            if o3 == 0 and on_segment(p2, p1, q2):
                intersections += 1

        return intersections % 2 == 1

    def find_closest_edge(self, test_point):
        """
        Finds the closest edge of the shape to a given point.

        Args:
            test_point (PointMass): The point to test against.

        Returns:
            tuple: A tuple containing (edge_point1, edge_point2, closest_point_on_segment)
                Returns (None, None, None) if no edge is found.
        """
        min_dist_sq = float('inf')
        closest_edge = None
        closest_point_on_edge = None

        if len(self.points) < 2:
            return None, None, None

        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]

            # Vector from p1 to p2
            line_vec_x = p2.x - p1.x
            line_vec_y = p2.y - p1.y


            test_x = test_point.x - p1.x
            test_y = test_point.y - p1.y

            if PERIODIC:
                line_vec_x = pbc_operator(line_vec_x, DEFAULT_WIDTH)
                line_vec_y = pbc_operator(line_vec_y, DEFAULT_HEIGHT)
                test_x = pbc_operator(test_x, DEFAULT_WIDTH)
                test_y = pbc_operator(test_y, DEFAULT_HEIGHT)


            # Squared length of the edge
            line_len_sq = line_vec_x**2 + line_vec_y**2
            if line_len_sq < 0.00001:
                continue

            # Projection of vector (test_point - p1) onto the edge vector
            t = (test_x * line_vec_x + test_y * line_vec_y) / line_len_sq
            t = max(0, min(1, t))  # Clamp t to the segment [0, 1]

            # Closest point on the line segment
            closest_x = p1.x + t * line_vec_x
            closest_y = p1.y + t * line_vec_y

            if PERIODIC:
                closest_x = pbc_operator(closest_x, DEFAULT_WIDTH)
                closest_y = pbc_operator(closest_y, DEFAULT_HEIGHT)

            # Squared distance from the test point to the closest point on the segment
            dist_sq = (test_point.x - closest_x)**2 + (test_point.y - closest_y)**2

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_edge = (p1, p2)
                closest_point_on_edge = (closest_x, closest_y, t) # Also return t for weighting

        return closest_edge[0], closest_edge[1], closest_point_on_edge 