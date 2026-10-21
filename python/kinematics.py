import math

class DeltaKinematics:
    def __init__(self):
        # Geometric Specifications (mm)
        self.f = 259.04  # Base Triangle Side
        self.e = 120.00  # Effector Triangle Side
        self.rf = 130.00 # Upper Arm Length
        self.re = 315.00 # Lower Arm Length

        # Derived Constants
        # Circumradius of equilateral triangle: R = side / sqrt(3)
        self.R = self.f / (2 * math.sqrt(3))   # inradius: joints at side midpoints
        self.r = self.e / (2 * math.sqrt(3))
        
        # Angles of the 3 arms (Standard 120 degree spacing)
        # Arm 1 at 0 deg, Arm 2 at 120 deg, Arm 3 at 240 deg (standard convention)
        self.phi = [0, 120, 240]

    def inverse_kinematics(self, x, y, z):
        """
        Calculates the joint angles (theta1, theta2, theta3) for a given position (x, y, z).
        Returns tuple of angles in degrees defined from horizontal.
        Returns None if unreachable.
        """
        angles = []
        for i in range(3):
            # 1. Rotate point to local frame of the arm (where arm is on +X axis)
            phi_rad = math.radians(self.phi[i])
            cos_phi = math.cos(phi_rad)
            sin_phi = math.sin(phi_rad)

            # Local coordinates of P
            # x_local = x * cos(phi) + y * sin(phi)
            # y_local = -x * sin(phi) + y * cos(phi)
            # But we are rotating the Frame by phi, so the point coordinates rotates by -phi?
            # Yes. P_local = Rot(-phi) * P_global
            x_loc = x * cos_phi + y * sin_phi
            y_loc = -x * sin_phi + y * cos_phi
            z_loc = z

            # 2. Geometry in the arm plane
            # Effective horizontal offset due to effector radius
            # The wrist joint is at P_local + (r, 0, 0) relative to effector center?
            # No. Effector frame is aligned with Base frame (no rotation).
            # The effector joint for arm i is at angle phi relative to effector center.
            # In purely local frame (rotated), the effector joint is always at x = x_loc + r.
            # (Because we rotated everything so arm i is at angle 0).
            
            # Let J1 (Base Joint) be at (R, 0, 0) in local frame.
            # Let J2 (Elbow Joint) be at (R + rf * cos(theta), 0, rf * sin(theta)).
            # Let J3 (Wrist Joint) be at (x_loc + r, y_loc, z_loc).
            
            # Dist(J2, J3)^2 = re^2
            # (x_loc + r - (R + rf * cos(theta)))^2 + (y_loc)^2 + (z_loc - rf * sin(theta))^2 = re^2
            
            # Let x_prime = x_loc + r - R
            # y_prime = y_loc
            # z_prime = z_loc
            
            x_prime = x_loc + self.r - self.R
            y_prime = y_loc
            z_prime = z_loc
            
            # Equation: (x_prime - rf*cos_t)^2 + y_prime^2 + (z_prime - rf*sin_t)^2 = re^2
            # x_prime^2 - 2*x_prime*rf*cos_t + rf^2*cos^2_t + y_prime^2 + z_prime^2 - 2*z_prime*rf*sin_t + rf^2*sin^2_t = re^2
            # (x_prime^2 + y_prime^2 + z_prime^2) + rf^2 - 2*rf*(x_prime*cos_t + z_prime*sin_t) = re^2
            
            # C = (x_prime^2 + y_prime^2 + z_prime^2 + self.rf**2 - self.re**2) / (2 * self.rf)
            # x_prime * cos(theta) + z_prime * sin(theta) = C
            
            denom = 2 * self.rf
            val = (x_prime**2 + y_prime**2 + z_prime**2 + self.rf**2 - self.re**2)
            c_val = val / denom
            
            # Solution for a*cos(t) + b*sin(t) = c
            # Let r_sq = a^2 + b^2. If c^2 > r_sq, no solution.
            a = x_prime
            b = z_prime
            
            r_sq = a**2 + b**2
            if r_sq == 0:
                return None # Singularity or congruent? Unlikely if geometry is valid.
                
            c_sq = c_val**2
            if c_sq > r_sq:
                return None # Unreachable
            
            # Solve: theta = alpha +/- beta
            # alpha = atan2(b, a)
            # beta = acos(c / sqrt(a^2 + b^2))
            
            # Note: We want the solution where the elbow is "up" or "out" depending on configuration?
            # Usually Delta robots have two solutions. 
            # Solution 1: "Knees out" (standard operations).
            # Solution 2: "Knees in" (often mechanically impossible or inverted).
            # We typically want the larger value of z for the elbow? Or specific branch?
            # Let's compute both and see.
            
            dist = math.sqrt(r_sq)
            # cos(theta - alpha) = c / dist
            
            # alpha is angle of vector (a, b) = (x_prime, z_prime)
            alpha = math.atan2(b, a) 
            
            arg = c_val / dist
            # Clamp to -1, 1 to handle floating point noise if very close
            arg = max(-1.0, min(1.0, arg))
            
            beta = math.acos(arg)
            
            # Two solutions rads: alpha + beta, alpha - beta
            t1 = alpha + beta
            t2 = alpha - beta
            
            # Convert to degrees
            deg1 = math.degrees(t1)
            deg2 = math.degrees(t2)
            
            # We select deg1 (alpha + beta) for the standard "elbow out" configuration.
            # In our test case (Home), deg1 is approx -25 deg (pointing down/out),
            # while deg2 is approx -185 deg (pointing backwards/in).
            
            angles.append(deg1)
            
        return tuple(angles)
