"""
Point mass physics implementation using Verlet integration.
"""

import math
from ..utils import pbc_operator
from ..config import PERIODIC, DEFAULT_WIDTH, DEFAULT_HEIGHT

class PointMass:
    """
    A point mass represents a single point in space with mass and velocity.
    Forces can act upon it to affect its motion, but it has no collision of its own.
    Uses Verlet integration for better stability and energy conservation.
    """
    
    def __init__(self, x, y, mass=1.0, drag_coefficient=0.0):
        """
        Initialize a point mass.
        
        Args:
            x (float): Initial x position
            y (float): Initial y position
            mass (float): Mass of the point (default 1.0)
            drag_coefficient (float): Viscous drag coefficient (default 0.0)
        """
        # Current position
        self.x = x
        self.y = y
        
        # Previous position (for Verlet integration)
        self.prev_x = x
        self.prev_y = y
        
        # Velocity (derived from position difference in Verlet)
        self.vx = 0.0
        self.vy = 0.0
        
        # Mass
        self.mass = mass
        
        # Drag coefficient for viscous drag
        self.drag_coefficient = drag_coefficient
        
        # Accumulated forces for this frame
        self.force_x = 0.0
        self.force_y = 0.0
        
        # Flag to handle first integration step
        self.first_step = True
    
    def apply_force(self, fx, fy):
        """
        Apply a force to this point mass.
        
        Args:
            fx (float): Force in x direction
            fy (float): Force in y direction
        """
        self.force_x += fx
        self.force_y += fy
    
    def apply_drag_force(self, drag_type="linear"):
        """
        Apply viscous drag force based on current velocity.
        
        Args:
            drag_type (str): Type of drag - "linear" or "quadratic"
        """
        if self.drag_coefficient <= 0:
            return
        
        if drag_type == "linear":
            # Linear drag: F_drag = -k * v
            drag_fx = -self.drag_coefficient * self.vx
            drag_fy = -self.drag_coefficient * self.vy
        elif drag_type == "quadratic":
            # Quadratic drag: F_drag = -k * v * |v|
            speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
            if speed > 0.001:  # Avoid division by zero
                drag_fx = -self.drag_coefficient * self.vx * speed
                drag_fy = -self.drag_coefficient * self.vy * speed
            else:
                drag_fx = drag_fy = 0.0
        else:
            drag_fx = drag_fy = 0.0
        
        self.apply_force(drag_fx, drag_fy)
    
    def set_drag_coefficient(self, drag_coefficient):
        """Set the drag coefficient."""
        self.drag_coefficient = max(0.0, drag_coefficient)
    
    def update(self, dt):
        """
        Update the point mass position using Verlet integration.
        Verlet integration: x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt^2
        
        Args:
            dt (float): Time step
        """
        # Apply Newton's second law: F = ma, so a = F/m
        ax = self.force_x / self.mass
        ay = self.force_y / self.mass

        
        if self.first_step:
            # For the first step, use Euler integration to establish previous position
            # Store current position as previous
            self.prev_x = self.x
            self.prev_y = self.y
            
            # Update position using Euler: p = p + v*dt + 0.5*a*dt^2 for the first time step
            self.x += self.vx * dt + 0.5 * ax * dt * dt
            self.y += self.vy * dt + 0.5 * ay * dt * dt
            
            # Update velocity: v = v + a*dt
            self.vx += ax * dt
            self.vy += ay * dt
            
            self.first_step = False
        else:
            # Verlet integration
            # Store current position
            temp_x = self.x
            temp_y = self.y
            
            # Calculate new position using standard Verlet
            #x(t+Δt) = 2x(t) - x(t-Δt) + a(t)Δt²
            self.x = 2.0 * self.x - self.prev_x + ax * dt * dt
            self.y = 2.0 * self.y - self.prev_y + ay * dt * dt
            
            # Update previous position
            self.prev_x = temp_x
            self.prev_y = temp_y
            
            # Derive velocity from position difference: v(t) = [x(t) - x(t-Δt)] / Δt
            self.vx = (self.x - self.prev_x) / dt if not PERIODIC else pbc_operator((self.x - self.prev_x), DEFAULT_WIDTH) / dt
            self.vy = (self.y - self.prev_y) / dt if not PERIODIC else pbc_operator((self.y - self.prev_y), DEFAULT_HEIGHT) / dt

        # Clear forces for next frame
        self.force_x = 0.0
        self.force_y = 0.0
    
    def get_position(self):
        """Get the current position as a tuple."""
        return (self.x, self.y)
    
    def get_velocity(self):
        """Get the current velocity as a tuple."""
        return (self.vx, self.vy)
    
    def set_position(self, x, y):
        """Set the position directly."""
        # When setting position directly, update previous position to maintain velocity
        self.prev_x = self.x
        self.prev_y = self.y
        self.x = x
        self.y = y
    
    def set_velocity(self, vx, vy):
        """Set the velocity directly by adjusting previous position."""
        self.vx = vx
        self.vy = vy
        # Adjust previous position to achieve desired velocity: prev = current - v*dt
        # Note: This assumes a standard dt, might need adjustment in actual use
        if not self.first_step:
            dt = 1.0/60.0  # Assume standard frame rate for velocity setting
            self.prev_x = self.x - vx * dt
            self.prev_y = self.y - vy * dt 