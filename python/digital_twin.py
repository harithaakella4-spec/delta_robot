import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
import numpy as np
import math
import time
import argparse
from python.kinematics import DeltaKinematics
from python.controller import DeltaRobot
from threading import Thread

def main():
    # === Command Line Arguments ===
    parser = argparse.ArgumentParser(description='Delta Robot Digital Twin')
    parser.add_argument('--mode', type=str, default='sim', choices=['sim', 'arduino'],
                        help='Interface mode: "sim" for simulation, "arduino" for real hardware')
    parser.add_argument('--port', type=str, default='COM3',
                        help='COM port for Arduino (e.g., COM3, /dev/ttyUSB0)')
    args = parser.parse_args()
    
    # 1. Instantiate Controller
    print(f"Starting in {args.mode.upper()} mode...")
    if args.mode == 'arduino':
        print(f"Using port: {args.port}")
        robot = DeltaRobot(interface_type='arduino', port=args.port)
    else:
        robot = DeltaRobot(interface_type='sim')
    
    if not robot.connect():
        print("ERROR: Failed to connect to robot!")
        return
    
    # Auto-home on startup
    print("Homing robot...")
    if not robot.home(use_safe_path=False):
        print("WARNING: Homing failed! Proceeding anyway for testing.")
        if robot.current_pos is None:
            # Fallback for visualization so it doesn't crash
            print("Setting default position (0, 0, -300) for visualization.")
            robot.current_pos = (0, 0, -300)

    # 2. Visualization Setup
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, bottom=0.35)
    
    # Grid/View limits
    limit = 350
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-500, 100)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    
    # Dynamic Title with Mode Indicator
    mode_str = f"[{args.mode.upper()}]" if args.mode == 'sim' else f"[ARDUINO @ {args.port}]"
    ax.set_title(f'Delta Robot Digital Twin {mode_str}')

    # Plot Objects
    base_lines = []
    effector_lines = []
    upper_arms = []
    lower_arms = []
    
    # Helper: Re-use the plot update logic, but now based on Robot State or Target
    # We will let the Sliders act as "Target Selector"
    # And the Robot Plot will track the "Current Real/Sim Position"
    
    # Initial Geometry Calculation Wrapper
    dk = robot.kinematics
    
    # Draw Base (Static)
    def draw_base():
        joints_B = []
        for phi in dk.phi:
            phi_rad = math.radians(phi)
            jx = dk.R * math.cos(phi_rad)
            jy = dk.R * math.sin(phi_rad)
            joints_B.append((jx, jy, 0))
        
        bx = [p[0] for p in joints_B] + [joints_B[0][0]]
        by = [p[1] for p in joints_B] + [joints_B[0][1]]
        bz = [p[2] for p in joints_B] + [joints_B[0][2]]
        ax.plot(bx, by, bz, 'k-', linewidth=2, label='Base')
        return joints_B

    joints_B = draw_base()
    
    # Dynamic Parts Holders
    eff_line, = ax.plot([], [], [], 'b-', linewidth=2, label='Effector')
    for _ in range(3):
        upper_arms.append(ax.plot([], [], [], 'g-', linewidth=2)[0])
        lower_arms.append(ax.plot([], [], [], 'g-', linewidth=2)[0])

    def update_plot(x, y, z):
        # Calculate joints for P(x,y,z)
        angles = dk.inverse_kinematics(x, y, z)
        if angles is None:
            return False # Invalid
            
        # Recalculate Joint Positions (Copy of logic from previous step)
        # We really should have exposed forward geometry in Kinematics class, but this is fine for now.
        
        jE = []
        jW = []
        
        for i, (phi, theta) in enumerate(zip(dk.phi, angles)):
            phi_rad = math.radians(phi)
            theta_rad = math.radians(theta)
            
            # Elbow
            e_loc_x = dk.R + dk.rf * math.cos(theta_rad)
            e_loc_z = dk.rf * math.sin(theta_rad)
            
            ex = e_loc_x * math.cos(phi_rad)
            ey = e_loc_x * math.sin(phi_rad)
            ez = e_loc_z
            jE.append((ex, ey, ez))
            
            # Wrist
            wx = x + dk.r * math.cos(phi_rad)
            wy = y + dk.r * math.sin(phi_rad)
            wz = z
            jW.append((wx, wy, wz))
            
        # Draw
        # Effector
        ex = [p[0] for p in jW] + [jW[0][0]]
        ey = [p[1] for p in jW] + [jW[0][1]]
        ez = [p[2] for p in jW] + [jW[0][2]]
        eff_line.set_data(ex, ey)
        eff_line.set_3d_properties(ez)
        
        # Arms
        for i in range(3):
            upper_arms[i].set_data([joints_B[i][0], jE[i][0]], [joints_B[i][1], jE[i][1]])
            upper_arms[i].set_3d_properties([joints_B[i][2], jE[i][2]])
            
            lower_arms[i].set_data([jE[i][0], jW[i][0]], [jE[i][1], jW[i][1]])
            lower_arms[i].set_3d_properties([jE[i][2], jW[i][2]])
            
        return True

    # Sliders (Controls for Target)
    axcolor = 'lightgoldenrodyellow'
    ax_z = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)
    ax_y = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
    ax_x = plt.axes([0.1, 0.20, 0.65, 0.03], facecolor=axcolor)

    s_x = Slider(ax_x, 'X', -200, 200, valinit=0)
    s_y = Slider(ax_y, 'Y', -200, 200, valinit=0)
    s_z = Slider(ax_z, 'Z', -500, -100, valinit=-300)

    # Ghost Function (Preview Target)
    # We can use a scatter point to show target
    target_dot, = ax.plot([0], [0], [-300], 'rx')

    def slider_update(val):
        tx, ty, tz = s_x.val, s_y.val, s_z.val
        target_dot.set_data([tx], [ty])
        target_dot.set_3d_properties([tz])
        fig.canvas.draw_idle()
    
    s_x.on_changed(slider_update)
    s_y.on_changed(slider_update)
    s_z.on_changed(slider_update)

    # Buttons
    ax_home = plt.axes([0.8, 0.15, 0.1, 0.04])
    b_home = Button(ax_home, 'Home')
    
    ax_move = plt.axes([0.8, 0.10, 0.1, 0.04])
    b_move = Button(ax_move, 'Move to Target')

    def on_home(event):
        print("Home Clicked")
        robot.home()
        # Update Plot to Robot Position
        rx, ry, rz = robot.current_pos
        update_plot(rx, ry, rz)
        # Sync sliders
        s_x.set_val(rx)
        s_y.set_val(ry)
        s_z.set_val(rz)
        fig.canvas.draw_idle()

    def on_move(event):
        target = (s_x.val, s_y.val, s_z.val)
        print(f"Executing Move to {target}")
        
        # We need to run the loop to animate the plot
        # Problem: Matplotlib event loop blocks.
        # Solution: Use simple generator loops or time.sleep + fig.canvas.flush_events()
        # CAUTION: time.sleep might freeze UI if not careful.
        
        path = robot.planner.plan_linear_move(robot.current_pos, target, velocity=200.0) # Faster
        
        for param in path:
            robot.current_pos = param
            robot.interface.move_joints(*robot.kinematics.inverse_kinematics(*param))
            
            update_plot(*robot.current_pos)
            fig.canvas.draw()
            fig.canvas.flush_events()
            # time.sleep(0.01) # Small delay for visuals
            
    b_home.on_clicked(on_home)
    b_move.on_clicked(on_move)

    # Initial Draw
    update_plot(*robot.current_pos)
    plt.show()

if __name__ == "__main__":
    main()
