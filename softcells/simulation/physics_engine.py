"""
Pure physics engine for soft body simulation.
This module contains no rendering or visualization code.
"""

from ..core import PointMass
from ..shapes import CircleShape
from ..config import (
    DEFAULT_DT, DEFAULT_GRAVITY, DEFAULT_GLOBAL_DRAG_COEFFICIENT, 
    DEFAULT_DRAG_TYPE, GLOBAL_PRESSURE_AMOUNT, PERIODIC
)
from ..utils import pbc_operator
from .collision_handler import CollisionHandler


class PhysicsEngine:
    """
    Pure physics simulation engine with no rendering dependencies.
    Handles all physics calculations, forces, and object interactions.
    """
    
    def __init__(self, world_width, world_height):
        """
        Initialize the physics engine.
        
        Args:
            world_width (float): Width of the physics world
            world_height (float): Height of the physics world
        """
        # World boundaries
        self.world_width = world_width
        self.world_height = world_height
        
        # Physics parameters
        self.dt = DEFAULT_DT
        self.gravity = DEFAULT_GRAVITY
        
        # Drag physics parameters
        self.global_drag_coefficient = DEFAULT_GLOBAL_DRAG_COEFFICIENT
        self.drag_type = DEFAULT_DRAG_TYPE
        self.drag_enabled = True
        
        # Simulation control
        self.paused = False
        self.step_next_frame = False
        
        # Physics objects
        self.points = []  # Individual point masses
        self.shapes = []  # Shape objects (which contain point masses)
        
        # Collision handling
        self.collision_handler = CollisionHandler()
        
        # Initialize scene
        self.create_initial_scene()
    
    def create_initial_scene(self):
        """Create initial physics objects for the simulation."""
        pass
        # Create a circle shape with default settings
        # circle1 = CircleShape(
        #     500, 100, 50, 
        #     num_points=20, 
        #     point_mass=1.0, 
        #     pressure=GLOBAL_PRESSURE_AMOUNT,
        #     spring_stiffness=1150.0, 
        #     spring_damping=10.0, 
        #     drag_coefficient=self.global_drag_coefficient, 
        #     drag_type=self.drag_type
        # )
        # self.shapes.append(circle1)
    
    def add_point(self, x, y, mass=1.0):
        """
        Add a point mass to the simulation.
        
        Args:
            x (float): X position
            y (float): Y position
            mass (float): Mass of the point
            
        Returns:
            PointMass: The created point mass
        """
        point = PointMass(x, y, mass, self.global_drag_coefficient)
        self.points.append(point)
        return point
    
    def add_circle_shape(self, center_x, center_y, radius, num_points=50, 
                        point_mass=1.0, pressure=None, spring_stiffness=1150.0, 
                        spring_damping=10.0):
        """
        Add a circle shape to the simulation.
        
        Args:
            center_x (float): X position of center
            center_y (float): Y position of center
            radius (float): Radius of the circle
            num_points (int): Number of points on the circle
            point_mass (float): Mass of each point
            pressure (float): Pressure amount (None for default)
            spring_stiffness (float): Spring stiffness
            spring_damping (float): Spring damping
            
        Returns:
            CircleShape: The created circle shape
        """
        if pressure is None:
            pressure = GLOBAL_PRESSURE_AMOUNT
            
        circle = CircleShape(
            center_x, center_y, radius,
            num_points=num_points,
            point_mass=point_mass,
            pressure=pressure,
            spring_stiffness=spring_stiffness,
            spring_damping=spring_damping,
            drag_coefficient=self.global_drag_coefficient,
            drag_type=self.drag_type
        )
        self.shapes.append(circle)
        return circle
    
    def remove_all_points(self):
        """Remove all individual points from the simulation."""
        self.points.clear()
    
    def remove_all_shapes(self):
        """Remove all shapes from the simulation."""
        self.shapes.clear()
    
    def reset_simulation(self):
        """Reset the simulation to initial state."""
        self.points.clear()
        self.shapes.clear()
        self.create_initial_scene()
    
    def set_gravity(self, gravity):
        """Set the gravity value."""
        self.gravity = gravity
    
    def set_drag_properties(self, coefficient, drag_type="linear", enabled=True):
        """
        Set global drag properties.
        
        Args:
            coefficient (float): Drag coefficient
            drag_type (str): "linear" or "quadratic"
            enabled (bool): Whether drag is enabled
        """
        self.global_drag_coefficient = coefficient
        self.drag_type = drag_type
        self.drag_enabled = enabled
        
        # Update all shapes
        for shape in self.shapes:
            shape.set_drag_properties(coefficient, drag_type)
            shape.enable_drag(enabled)
        
        # Update all individual points
        for point in self.points:
            point.set_drag_coefficient(coefficient)
    
    def set_pressure_for_all_shapes(self, pressure_amount):
        """Set pressure for all shapes."""
        for shape in self.shapes:
            shape.set_pressure(pressure_amount)
    
    def toggle_pressure_for_all_shapes(self):
        """Toggle pressure physics for all shapes."""
        for shape in self.shapes:
            shape.enable_pressure(not shape.pressure_enabled)
    
    def set_spring_properties_for_all_shapes(self, stiffness=None, damping=None):
        """Set spring properties for all shapes."""
        for shape in self.shapes:
            current_stiffness = shape.spring_stiffness if stiffness is None else stiffness
            current_damping = shape.spring_damping if damping is None else damping
            shape.set_spring_properties(current_stiffness, current_damping)
    
    def toggle_springs_for_all_shapes(self):
        """Toggle spring physics for all shapes."""
        for shape in self.shapes:
            shape.enable_springs(not shape.springs_enabled)
    
    def update_physics(self):
        """Update all physics objects with forces and motion."""
        # Update individual point masses
        for point in self.points:
            # Apply gravity
            point.apply_force(0, self.gravity * point.mass)
            
            # Apply drag force if enabled
            if self.drag_enabled:
                point.apply_drag_force(self.drag_type)
            
            # Update physics
            point.update(self.dt)
            
            # Handle boundary conditions (bounce off walls)
            self._handle_boundary_collision(point)
        
        # Update shapes (which contain point masses)
        for shape in self.shapes:
            # Apply gravity to all points in the shape
            shape.apply_force_to_all(0, self.gravity)
            
            # Apply drag forces to all points in the shape
            shape.apply_drag_forces()
            
            # Apply spring forces between connected points
            shape.apply_spring_forces()
            
            # Apply pressure forces to maintain shape integrity
            shape.apply_pressure_forces()
            
            # Update physics for all points in the shape
            shape.update_all(self.dt)
            
            # Handle boundary collisions for all points in the shape
            for point in shape.get_points():
                self._handle_boundary_collision(point)

        # Handle shape-to-shape collisions
        self.collision_handler.handle_collisions(self.shapes)
    
    def _handle_boundary_collision(self, point):
        """Handle collision with world boundaries for a single point."""
        if not PERIODIC:
            if point.x < 0:
                point.x = 0
                point.vx = -point.vx * 0.8  # Some energy loss on bounce
            elif point.x > self.world_width:
                point.x = self.world_width
                point.vx = -point.vx * 0.8
            
            if point.y < 0:
                point.y = 0
                point.vy = -point.vy * 0.8
            elif point.y > self.world_height:
                point.y = self.world_height
                point.vy = -point.vy * 0.8
        else:
            # Periodic boundary conditions
            point.x = point.x % self.world_width
            point.y = point.y % self.world_height

    def step(self):
        """Advance the simulation by one time step."""
        self.update_physics()
    
    def get_simulation_state(self):
        """
        Get the current state of the simulation for rendering or analysis.
        
        Returns:
            dict: Dictionary containing all simulation state information
        """
        return {
            'points': [
                {
                    'x': point.x,
                    'y': point.y,
                    'vx': point.vx,
                    'vy': point.vy,
                    'mass': point.mass,
                    'drag_coefficient': point.drag_coefficient
                }
                for point in self.points
            ],
            'shapes': [
                {
                    'points': [
                        {
                            'x': point.x,
                            'y': point.y,
                            'vx': point.vx,
                            'vy': point.vy,
                            'mass': point.mass
                        }
                        for point in shape.get_points()
                    ],
                    'springs': [
                        {
                            'point1_idx': shape.get_points().index(spring.point1),
                            'point2_idx': shape.get_points().index(spring.point2),
                            'rest_length': spring.rest_length,
                            'current_length': spring.get_current_length(),
                            'stretch_ratio': spring.get_stretch_ratio()
                        }
                        for spring in shape.springs
                    ] if hasattr(shape, 'springs') else [],
                    'color': getattr(shape, 'color', (255, 255, 255)),
                    'pressure_enabled': getattr(shape, 'pressure_enabled', False),
                    'pressure_amount': getattr(shape, 'pressure_amount', 0),
                    'scalar_pressure': getattr(shape, 'scalar_pressure', 0),
                    'current_volume': getattr(shape, 'current_volume', 0),
                    'springs_enabled': getattr(shape, 'springs_enabled', False),
                    'spring_stiffness': getattr(shape, 'spring_stiffness', 0),
                    'spring_damping': getattr(shape, 'spring_damping', 0),
                    'drag_enabled': getattr(shape, 'drag_enabled', False),
                    'drag_coefficient': getattr(shape, 'drag_coefficient', 0),
                    'drag_type': getattr(shape, 'drag_type', 'linear')
                }
                for shape in self.shapes
            ],
            'physics_params': {
                'dt': self.dt,
                'gravity': self.gravity,
                'global_drag_coefficient': self.global_drag_coefficient,
                'drag_type': self.drag_type,
                'drag_enabled': self.drag_enabled,
                'world_width': self.world_width,
                'world_height': self.world_height
            }
        }
