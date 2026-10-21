"""
Workspace Analyzer for Delta-X Puly S Robot
Calculates and visualizes the reachable 3D workspace considering motor angle limits.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from python.kinematics import DeltaKinematics

class WorkspaceAnalyzer:
    def __init__(self):
        self.kinematics = DeltaKinematics()
        
        # Motor angle limits (degrees) - based on physical endstops
        # Adjust these based on your actual hardware limits
        self.angle_min = -90.0  # Arm pointing down
        self.angle_max = 90.0   # Arm pointing up (usually limited by structure)
        
        # Workspace sampling resolution (mm)
        self.resolution = 10.0
        
        # Reachable points storage
        self.reachable_points = []
        
    def is_within_limits(self, angles):
        """Check if all joint angles are within physical limits."""
        if angles is None:
            return False
        for angle in angles:
            if angle < self.angle_min or angle > self.angle_max:
                return False
        return True
    
    def analyze_workspace(self, x_range=(-200, 200), y_range=(-200, 200), z_range=(-450, -50)):
        """
        Scan 3D space and find all reachable points.
        
        Args:
            x_range: (min, max) for X axis in mm
            y_range: (min, max) for Y axis in mm
            z_range: (min, max) for Z axis in mm (negative = below base)
        """
        print("Analyzing workspace...")
        print(f"  X range: {x_range}")
        print(f"  Y range: {y_range}")
        print(f"  Z range: {z_range}")
        print(f"  Resolution: {self.resolution} mm")
        print(f"  Angle limits: [{self.angle_min}°, {self.angle_max}°]")
        
        self.reachable_points = []
        
        x_vals = np.arange(x_range[0], x_range[1] + self.resolution, self.resolution)
        y_vals = np.arange(y_range[0], y_range[1] + self.resolution, self.resolution)
        z_vals = np.arange(z_range[0], z_range[1] + self.resolution, self.resolution)
        
        total_points = len(x_vals) * len(y_vals) * len(z_vals)
        checked = 0
        
        for z in z_vals:
            for y in y_vals:
                for x in x_vals:
                    angles = self.kinematics.inverse_kinematics(x, y, z)
                    if self.is_within_limits(angles):
                        self.reachable_points.append((x, y, z))
                    
                    checked += 1
                    if checked % 10000 == 0:
                        print(f"  Progress: {checked}/{total_points} ({100*checked/total_points:.1f}%)")
        
        print(f"\nAnalysis complete!")
        print(f"  Total points checked: {total_points}")
        print(f"  Reachable points: {len(self.reachable_points)}")
        
        return self.reachable_points
    
    def get_bounds(self):
        """Get the bounding box of the reachable workspace."""
        if not self.reachable_points:
            return None
            
        points = np.array(self.reachable_points)
        
        bounds = {
            'x_min': points[:, 0].min(),
            'x_max': points[:, 0].max(),
            'y_min': points[:, 1].min(),
            'y_max': points[:, 1].max(),
            'z_min': points[:, 2].min(),
            'z_max': points[:, 2].max(),
        }
        
        return bounds
    
    def visualize(self, show_slices=True):
        """Visualize the workspace as a 3D scatter plot."""
        if not self.reachable_points:
            print("No data to visualize. Run analyze_workspace() first.")
            return
            
        points = np.array(self.reachable_points)
        
        fig = plt.figure(figsize=(12, 5))
        
        # 3D Scatter
        ax1 = fig.add_subplot(121, projection='3d')
        
        # Color by Z height
        colors = points[:, 2]
        scatter = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                             c=colors, cmap='viridis', s=1, alpha=0.5)
        
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.set_title('Delta Robot Workspace')
        
        # Add base triangle for reference
        dk = self.kinematics
        import math
        base_x = [dk.R * math.cos(math.radians(phi)) for phi in dk.phi]
        base_y = [dk.R * math.sin(math.radians(phi)) for phi in dk.phi]
        base_x.append(base_x[0])
        base_y.append(base_y[0])
        ax1.plot(base_x, base_y, [0]*4, 'r-', linewidth=2, label='Base')
        
        plt.colorbar(scatter, ax=ax1, label='Z Height (mm)')
        
        # 2D Top-down view (XY slice at mid-Z)
        ax2 = fig.add_subplot(122)
        
        bounds = self.get_bounds()
        mid_z = (bounds['z_min'] + bounds['z_max']) / 2
        
        # Get points near mid_z
        tolerance = self.resolution * 1.5
        slice_mask = np.abs(points[:, 2] - mid_z) < tolerance
        slice_points = points[slice_mask]
        
        if len(slice_points) > 0:
            ax2.scatter(slice_points[:, 0], slice_points[:, 1], s=5, alpha=0.7)
        
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_title(f'XY Slice at Z = {mid_z:.0f} mm')
        ax2.set_aspect('equal')
        ax2.grid(True)
        
        plt.tight_layout()
        plt.show()


def main():
    analyzer = WorkspaceAnalyzer()
    
    # Set motor limits based on your endstops
    # Typical delta robots have limits around -30 to +90 degrees
    # Adjust based on your Puly S configuration
    analyzer.angle_min = -30.0  # Lower limit (arm pointing down-ish)
    analyzer.angle_max = 90.0   # Upper limit (arm pointing up)
    
    # Analyze with finer resolution for better accuracy (takes longer)
    analyzer.resolution = 15.0  # mm
    
    # Run analysis
    analyzer.analyze_workspace(
        x_range=(-200, 200),
        y_range=(-200, 200),
        z_range=(-450, -100)
    )
    
    # Print bounds
    bounds = analyzer.get_bounds()
    if bounds:
        print("\n=== WORKSPACE BOUNDS ===")
        print(f"  X: [{bounds['x_min']:.1f}, {bounds['x_max']:.1f}] mm")
        print(f"  Y: [{bounds['y_min']:.1f}, {bounds['y_max']:.1f}] mm")
        print(f"  Z: [{bounds['z_min']:.1f}, {bounds['z_max']:.1f}] mm")
        
        # Calculate approximate volume
        dx = bounds['x_max'] - bounds['x_min']
        dy = bounds['y_max'] - bounds['y_min']
        dz = bounds['z_max'] - bounds['z_min']
        print(f"\n  Bounding Box Dimensions: {dx:.0f} x {dy:.0f} x {dz:.0f} mm")
    
    # Visualize
    analyzer.visualize()


if __name__ == "__main__":
    main()
