"""
Hardware Interfaces for Delta Robot Control
Provides abstraction layer between controller logic and physical/simulated hardware.
"""

import time

class RobotInterface:
    """Base class for robot hardware interfaces."""
    
    def connect(self):
        raise NotImplementedError
        
    def disconnect(self):
        raise NotImplementedError
        
    def is_connected(self):
        raise NotImplementedError
        
    def move_joints(self, theta1, theta2, theta3):
        """
        Move robot to specific joint angles.
        Args:
           theta1, theta2, theta3: Angles in degrees.
        """
        raise NotImplementedError

    def homing_sequence(self):
        """
        Perform hardware homing routine 
        (e.g., move toward endstops until triggered, then zero).
        Returns True if successful, False otherwise.
        """
        raise NotImplementedError
        
    def read_endstops(self):
        """
        Read current endstop states.
        Returns: dict with keys 'motor1', 'motor2', 'motor3' (True = triggered)
        """
        raise NotImplementedError
    
    def emergency_stop(self):
        """Immediately stop all motors."""
        raise NotImplementedError


class SimulationInterface(RobotInterface):
    """Simulated robot interface for testing without hardware."""
    
    def __init__(self):
        self._connected = False
        self.current_angles = (0, 0, 0)
        self._endstops = {'motor1': False, 'motor2': False, 'motor3': False}
        print("[SIM] Simulation Interface Initialized")

    def connect(self):
        self._connected = True
        print("[SIM] Simulation Connected")
        return True
        
    def disconnect(self):
        self._connected = False
        print("[SIM] Simulation Disconnected")
        
    def is_connected(self):
        return self._connected

    def move_joints(self, t1, t2, t3):
        if not self._connected:
            print("[SIM] Warning: Not connected")
            return False
        self.current_angles = (t1, t2, t3)
        return True
        
    def homing_sequence(self):
        print("[SIM] Simulating Homing Sequence...")
        time.sleep(0.5)  # Fake delay
        self.current_angles = (0, 0, 0)
        print("[SIM] Homing Complete")
        return True
        
    def read_endstops(self):
        return self._endstops.copy()
        
    def emergency_stop(self):
        print("[SIM] EMERGENCY STOP")
        return True


class ArduinoInterface(RobotInterface):
    """
    Interface for Arduino Mega with RAMPS 1.4 board.
    Communicates via Serial (pyserial).
    
    Protocol:
        - Move:     "M1 A<angle1> B<angle2> C<angle3>\\n"  -> "OK\\n"
        - Home:     "G28\\n"                               -> "OK\\n"
        - Stop:     "M0\\n"                                -> "OK\\n"
        - Status:   "?\\n"                                 -> "E1:<0/1> E2:<0/1> E3:<0/1>\\n"
    """
    
    def __init__(self, port='COM3', baud=115200, timeout=2.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial = None
        self._connected = False
        
    def connect(self):
        """Establish serial connection to Arduino."""
        try:
            import serial
        except ImportError:
            print("[ARDUINO] ERROR: pyserial not installed. Run: pip install pyserial")
            return False
            
        try:
            print(f"[ARDUINO] Connecting to {self.port} at {self.baud} baud...")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            
            # Wait for Arduino to reset (bootloader delay)
            print("[ARDUINO] Waiting for Arduino bootloader...")
            time.sleep(2.5)
            
            # Clear any startup messages
            self.serial.reset_input_buffer()
            
            # Test connection with status query
            self.serial.write(b"?\n")
            response = self.serial.readline().decode().strip()
            
            if response:
                print(f"[ARDUINO] Connected! Response: {response}")
                self._connected = True
                return True
            else:
                print("[ARDUINO] Warning: No response from Arduino")
                self._connected = True  # Still mark as connected
                return True
                
        except Exception as e:
            print(f"[ARDUINO] Connection failed: {e}")
            self._connected = False
            return False
            
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[ARDUINO] Disconnected")
        self._connected = False
        
    def is_connected(self):
        return self._connected and self.serial and self.serial.is_open
        
    def _send_command(self, cmd, wait_for_ok=True):
        """
        Send command and optionally wait for OK response.
        Returns response string or None on failure.
        """
        if not self.is_connected():
            print("[ARDUINO] Error: Not connected")
            return None
            
        try:
            # Send command
            if not cmd.endswith('\n'):
                cmd += '\n'
            self.serial.write(cmd.encode())
            
            if wait_for_ok:
                # Wait for response
                response = self.serial.readline().decode().strip()
                return response
            return "SENT"
            
        except Exception as e:
            print(f"[ARDUINO] Command error: {e}")
            return None

    def move_joints(self, t1, t2, t3):
        """
        Send move command to Arduino.
        
        Args:
            t1, t2, t3: Joint angles in degrees
        """
        cmd = f"M1 A{t1:.2f} B{t2:.2f} C{t3:.2f}"
        response = self._send_command(cmd)
        
        if response and "OK" in response:
            return True
        else:
            print(f"[ARDUINO] Move failed. Response: {response}")
            return False
            
    def homing_sequence(self):
        """
        Send G28 homing command.
        Note: Arduino sends multiple lines during homing (e.g., "Homing...", "M1 homed", etc.)
        We need to read all lines until we get "OK" or "ERROR".
        """
        print("[ARDUINO] Sending homing command...")
        
        if not self.is_connected():
            print("[ARDUINO] Error: Not connected")
            return False
        
        try:
            self.serial.write(b"G28\n")
            
            # Read lines until we get OK or ERROR (with timeout)
            start_time = time.time()
            timeout = 90.0  # 90 seconds for homing (3 motors)
            
            while (time.time() - start_time) < timeout:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode().strip()
                    if line:
                        print(f"[ARDUINO] {line}")
                        
                        if "OK" in line:
                            print("[ARDUINO] Homing complete!")
                            return True
                        elif "ERROR" in line:
                            print("[ARDUINO] Homing failed!")
                            return False
                else:
                    time.sleep(0.1)  # Small delay to avoid busy-waiting
            
            print("[ARDUINO] Homing timeout!")
            return False
            
        except Exception as e:
            print(f"[ARDUINO] Homing error: {e}")
            return False
            
    def read_endstops(self):
        """
        Query endstop states from Arduino.
        Expected response: "E1:0 E2:1 E3:0" (0=open, 1=triggered)
        """
        response = self._send_command("?")
        
        endstops = {'motor1': False, 'motor2': False, 'motor3': False}
        
        if response:
            try:
                # Parse response like "E1:0 E2:1 E3:0"
                parts = response.split()
                for part in parts:
                    if part.startswith("E1:"):
                        endstops['motor1'] = part.split(":")[1] == "1"
                    elif part.startswith("E2:"):
                        endstops['motor2'] = part.split(":")[1] == "1"
                    elif part.startswith("E3:"):
                        endstops['motor3'] = part.split(":")[1] == "1"
            except Exception as e:
                print(f"[ARDUINO] Error parsing endstop response: {e}")
                
        return endstops
        
    def emergency_stop(self):
        """Send emergency stop command."""
        print("[ARDUINO] EMERGENCY STOP!")
        # Don't wait for OK - just send immediately
        response = self._send_command("M0", wait_for_ok=False)
        return True
