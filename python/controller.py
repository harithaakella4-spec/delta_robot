"""
Delta Robot Controller
High-level orchestration of kinematics, planning, and hardware interface.
"""

import time
from python.kinematics import DeltaKinematics
from python.planner import TrajectoryPlanner
import python.interfaces as interfaces

class DeltaRobot:
    """
    Main controller class for Delta Robot.
    Manages state, coordinates motion, and handles homing.
    """
    
    # Standard positions (mm)
    HOME_POSITION = (0, 0, -300)
    SAFE_Z_HEIGHT = -150  # Safe height to move to before XY motion
    
    def __init__(self, interface_type='sim', port='COM3'):
        self.kinematics = DeltaKinematics()
        self.planner = TrajectoryPlanner()
        
        if interface_type == 'arduino':
            self.interface = interfaces.ArduinoInterface(port=port)
        else:
            self.interface = interfaces.SimulationInterface()
            
        self.current_pos = None  # Unknown until homed
        self.is_homed = False
        self.is_moving = False
        
        # Motion parameters
        self.default_velocity = 100.0  # mm/s
        self.homing_velocity = 50.0    # mm/s (slower for safety)
        
    def connect(self):
        """Connect to the hardware interface."""
        return self.interface.connect()
        
    def disconnect(self):
        """Disconnect from hardware."""
        self.interface.disconnect()
        
    def home(self, use_safe_path=True):
        """
        Execute homing sequence.
        
        If use_safe_path=True and we know our current position,
        we first move to a safe height before going to home.
        
        Args:
            use_safe_path: If True, move through safe intermediate positions
        """
        print("=== HOMING SEQUENCE ===")
        
        # Step 1: If position is known, move to safe height first
        if use_safe_path and self.current_pos is not None:
            print(f"Current position known: {self.current_pos}")
            print(f"Moving to safe height Z={self.SAFE_Z_HEIGHT} first...")
            
            # Move Z up to safe height (keep X, Y same)
            safe_pos = (self.current_pos[0], self.current_pos[1], self.SAFE_Z_HEIGHT)
            self._execute_move(self.current_pos, safe_pos, self.homing_velocity)
            
            # Move X, Y to 0 at safe height
            center_pos = (0, 0, self.SAFE_Z_HEIGHT)
            self._execute_move(safe_pos, center_pos, self.homing_velocity)
            
            print("At safe center position")
        
        # Step 2: Execute hardware homing (endstop detection)
        # For simulation, this is instant. For real hardware, Arduino handles it.
        print("Executing hardware homing routine...")
        result = self.interface.homing_sequence()
        
        if not result:
            print("ERROR: Homing failed!")
            return False
        
        # Step 3: Set known home position
        self.current_pos = self.HOME_POSITION
        self.is_homed = True
        
        # Sync angles with interface
        angles = self.kinematics.inverse_kinematics(*self.current_pos)
        if angles:
            self.interface.move_joints(*angles)
            print(f"Home angles: θ1={angles[0]:.1f}°, θ2={angles[1]:.1f}°, θ3={angles[2]:.1f}°")
        
        print(f"=== HOMED at {self.HOME_POSITION} ===")
        return True
        
    def move_to(self, target_pos, velocity=None):
        """
        Move from current position to target position along a linear path.
        
        Args:
            target_pos: (x, y, z) tuple in mm
            velocity: Movement speed in mm/s (default: self.default_velocity)
            
        Returns:
            True if move completed successfully, False otherwise
        """
        if velocity is None:
            velocity = self.default_velocity
            
        if not self.is_homed:
            print("WARNING: Robot not homed! Position may be incorrect.")
            # Set current position to home for safety
            self.current_pos = self.HOME_POSITION
            
        if self.current_pos is None:
            print("ERROR: Current position unknown. Please home first.")
            return False
            
        return self._execute_move(self.current_pos, target_pos, velocity)
        
    def move_to_safe(self, target_pos, velocity=None):
        """
        Move to target using safe path: up -> across -> down
        Avoids potential collisions by moving at safe Z height.
        
        Args:
            target_pos: (x, y, z) tuple in mm
            velocity: Movement speed in mm/s
        """
        if velocity is None:
            velocity = self.default_velocity
            
        if self.current_pos is None:
            print("ERROR: Current position unknown. Please home first.")
            return False
            
        tx, ty, tz = target_pos
        cx, cy, cz = self.current_pos
        
        # Only use safe path if we're moving significantly in XY
        xy_distance = ((tx - cx)**2 + (ty - cy)**2)**0.5
        
        if xy_distance > 50:  # More than 50mm XY movement
            print(f"Using safe path for large XY move ({xy_distance:.1f}mm)")
            
            # Step 1: Move up to safe height
            if cz < self.SAFE_Z_HEIGHT:
                safe_start = (cx, cy, self.SAFE_Z_HEIGHT)
                if not self._execute_move(self.current_pos, safe_start, velocity):
                    return False
                    
            # Step 2: Move across at safe height
            safe_end = (tx, ty, self.SAFE_Z_HEIGHT)
            if not self._execute_move(self.current_pos, safe_end, velocity):
                return False
                
            # Step 3: Move down to target
            if not self._execute_move(self.current_pos, target_pos, velocity):
                return False
        else:
            # Direct move for small movements
            if not self._execute_move(self.current_pos, target_pos, velocity):
                return False
                
        return True
        
    def _execute_move(self, start_pos, end_pos, velocity):
        """
        Internal method to execute a linear move.
        Generates trajectory and sends to interface.
        """
        self.is_moving = True
        
        path = self.planner.plan_linear_move(start_pos, end_pos, velocity)
        
        for point in path:
            angles = self.kinematics.inverse_kinematics(*point)
            
            if angles is None:
                print(f"ERROR: Unreachable point in path: {point}")
                self.is_moving = False
                return False
                
            self.interface.move_joints(*angles)
            self.current_pos = point
            
            # Small delay for smooth motion visualization
            # In real hardware, the Arduino handles timing
            if isinstance(self.interface, interfaces.SimulationInterface):
                time.sleep(0.02)
                
        self.is_moving = False
        return True
        
    def get_status(self):
        """Get current robot status."""
        return {
            'connected': self.interface.is_connected(),
            'homed': self.is_homed,
            'moving': self.is_moving,
            'position': self.current_pos,
            'endstops': self.interface.read_endstops()
        }
        
    def emergency_stop(self):
        """Emergency stop - halt all motion immediately."""
        self.is_moving = False
        return self.interface.emergency_stop()


# Quick test
if __name__ == "__main__":
    print("=== Delta Robot Controller Test ===")
    
    robot = DeltaRobot(interface_type='sim')
    robot.connect()
    
    # Test homing
    robot.home()
    
    print(f"\nStatus: {robot.get_status()}")
    
    # Test move
    print("\nMoving to (50, 50, -350)...")
    robot.move_to((50, 50, -350))
    
    print(f"\nFinal position: {robot.current_pos}")
