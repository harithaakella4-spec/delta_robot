import numpy as np

class TrajectoryPlanner:
    def __init__(self):
        pass

    def plan_linear_move(self, start_pos, end_pos, velocity, time_step=0.05):
        """
        Generates a sequence of waypoints for a linear path in Cartesian space.
        
        Args:
            start_pos: (x, y, z) tuple
            end_pos: (x, y, z) tuple
            velocity: mm/s
            time_step: Time difference between waypoints (s)
            
        Returns:
            List of (x, y, z) tuples representing the path.
        """
        p0 = np.array(start_pos)
        p1 = np.array(end_pos)
        
        distance = np.linalg.norm(p1 - p0)
        
        if distance < 1e-3:
            return [end_pos]
            
        duration = distance / velocity
        
        # Number of steps
        # Ensure at least 2 points (start and end)
        num_steps = int(np.ceil(duration / time_step))
        if num_steps < 1:
            num_steps = 1
            
        timestamps = np.linspace(0, 1, num_steps + 1)
        
        # Simple Linear Interpolation (Constant Velocity)
        # TODO in future: Add Trapezoidal Velocity Profile (Accel/Decel)
        
        waypoints = []
        for t in timestamps:
            # Linear interp: p = p0 + t * (p1 - p0)
            point = p0 + t * (p1 - p0)
            waypoints.append(tuple(point))
            
        return waypoints
