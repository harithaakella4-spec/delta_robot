/*
 * Delta Robot Controller Firmware
 * For Arduino Mega 2560 with RAMPS 1.4 Shield
 * Using A4988 Stepper Motor Drivers
 *
 * Controls 3 stepper motors for a rotary delta robot.
 * Receives commands via Serial from Python controller.
 *
 * ============================================
 * A4988 DRIVER SETUP NOTES:
 * ============================================
 *
 * Microstepping Configuration (via RAMPS jumpers under each driver):
 *   MS1  MS2  MS3  |  Resolution
 *   ---------------------------------
 *   LOW  LOW  LOW  |  Full step
 *   HIGH LOW  LOW  |  1/2 step
 *   LOW  HIGH LOW  |  1/4 step
 *   HIGH HIGH LOW  |  1/8 step
 *   HIGH HIGH HIGH |  1/16 step  <-- Default for RAMPS (all 3 jumpers
 * installed)
 *
 * Current Limiting:
 *   - Adjust the potentiometer on each A4988
 *   - Vref = Imax * 8 * Rs  (Rs is usually 0.1 ohm)
 *   - For 1A motors: Vref ≈ 0.8V
 *   - For 1.5A motors: Vref ≈ 1.2V
 *   - MEASURE Vref between potentiometer center pin and GND
 *
 * Wiring:
 *   - Motor coils: 2A, 2B to one coil; 1A, 1B to second coil
 *   - If motor spins wrong direction: swap ONE coil pair (2A-2B or 1A-1B)
 *
 * ============================================
 * COMMAND PROTOCOL:
 * ============================================
 *   M1 A<angle1> B<angle2> C<angle3>  - Move motors to specified angles
 *   G28                                - Homing sequence (move to endstops)
 *   M0                                 - Emergency stop
 *   ?                                  - Query status (endstop states)
 *   M17                                - Enable motors
 *   M18                                - Disable motors
 *   RESET                              - Clear emergency stop
 *
 * Response:
 *   OK           - Command completed successfully
 *   ERROR:<msg>  - Command failed
 *   E1:X E2:X E3:X - Endstop states (0=open, 1=triggered)
 */

#include <AccelStepper.h>

// ============================================
// RAMPS 1.4 Pin Definitions for Arduino Mega
// ============================================

// Motor 1 (X axis on RAMPS) - First Delta Arm
#define M1_STEP_PIN 54
#define M1_DIR_PIN 55
#define M1_ENABLE_PIN 38
#define M1_ENDSTOP_PIN 3 // X_MIN on RAMPS

// Motor 2 (Y axis on RAMPS) - Second Delta Arm
#define M2_STEP_PIN 60
#define M2_DIR_PIN 61
#define M2_ENABLE_PIN 56
#define M2_ENDSTOP_PIN 14 // Y_MIN on RAMPS

// Motor 3 (Z axis on RAMPS) - Third Delta Arm
#define M3_STEP_PIN 46
#define M3_DIR_PIN 48
#define M3_ENABLE_PIN 62
#define M3_ENDSTOP_PIN 18 // Z_MIN on RAMPS

// ============================================
// A4988 / Stepper Configuration
// ============================================

// Stepper motor specifications
#define STEPS_PER_REV 200 // 1.8° stepper = 200 steps/rev (0.9° = 400)

// Microstepping (set by jumpers under A4988 on RAMPS)
// Options: 1 (full), 2 (half), 4, 8, or 16
#define MICROSTEPPING 16 // Default: all 3 jumpers installed = 1/16

// Gear ratio (if using belts/pulleys/gears between motor and arm)
#define GEAR_RATIO 1 // 1 = direct drive

// Calculated steps per degree of arm rotation
const float STEPS_PER_DEGREE =
    (float)(STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO) / 360.0;

// ============================================
// Motion Parameters
// ============================================

// Speed settings (steps per second)
// A4988 max reliable step rate: ~10,000 steps/sec with 16µs pulse width
#define MAX_SPEED 3000    // Normal operating speed
#define HOMING_SPEED 800  // Slower for homing (safety)
#define ACCELERATION 2000 // Steps/second² (smooths start/stop)

// A4988 timing (microseconds)
// Minimum pulse width: 1µs (A4988 spec)
// AccelStepper handles this internally, but we add safety margin
#define STEP_PULSE_WIDTH 2 // Microseconds (AccelStepper default is fine)

// ============================================
// Endstop Configuration
// ============================================

// Endstop logic level when triggered
// NC (Normally Closed) switches are SAFER - wire breaks = triggered
#define ENDSTOP_TRIGGERED LOW // LOW for NC switches, HIGH for NO switches

// Steps to back off after hitting endstop during homing
#define HOMING_BACKOFF 100 // About 11° with default settings

// Homing direction (-1 = toward endstop, +1 = away from endstop)
// Adjust if your motors are mounted "backwards"
#define M1_HOME_DIR -1
#define M2_HOME_DIR -1
#define M3_HOME_DIR -1

// ============================================
// Motor Direction Inversion
// ============================================
// Set to true if a motor rotates the wrong way
#define M1_INVERT_DIR false
#define M2_INVERT_DIR false
#define M3_INVERT_DIR false

// ============================================
// Stepper Objects
// ============================================

AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);

// ============================================
// Global State
// ============================================

bool isHomed = false;
bool emergencyStop = false;
String inputBuffer = "";

// Current angles (degrees) - tracked for position feedback
float currentAngle1 = 0;
float currentAngle2 = 0;
float currentAngle3 = 0;

// ============================================
// Setup
// ============================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port (needed for some boards)
  }

  // Configure enable pins (active LOW on A4988/RAMPS)
  pinMode(M1_ENABLE_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  pinMode(M3_ENABLE_PIN, OUTPUT);

  // Start with motors enabled
  enableMotors(true);

  // Configure endstop pins with internal pullup
  // This allows NC switches to work without external pullup resistors
  pinMode(M1_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(M2_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(M3_ENDSTOP_PIN, INPUT_PULLUP);

  // Configure motor parameters
  motor1.setMaxSpeed(MAX_SPEED);
  motor1.setAcceleration(ACCELERATION);
  motor1.setPinsInverted(M1_INVERT_DIR, false, false);

  motor2.setMaxSpeed(MAX_SPEED);
  motor2.setAcceleration(ACCELERATION);
  motor2.setPinsInverted(M2_INVERT_DIR, false, false);

  motor3.setMaxSpeed(MAX_SPEED);
  motor3.setAcceleration(ACCELERATION);
  motor3.setPinsInverted(M3_INVERT_DIR, false, false);

  // Startup message
  Serial.println("===================================");
  Serial.println("Delta Robot Controller v1.0");
  Serial.println("A4988 Drivers on RAMPS 1.4");
  Serial.println("===================================");
  Serial.print("Steps/degree: ");
  Serial.println(STEPS_PER_DEGREE);
  Serial.println("Commands: ?, G28, M0, M1 A# B# C#");
  Serial.println("Ready.");
}

// ============================================
// Main Loop
// ============================================

void loop() {
  // Process incoming serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // Run steppers (non-blocking motion updates)
  if (!emergencyStop) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
}

// ============================================
// Command Processing
// ============================================

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("M1 ")) {
    handleMoveCommand(cmd);
  } else if (cmd == "G28") {
    handleHomingCommand();
  } else if (cmd == "M0") {
    handleEmergencyStop();
  } else if (cmd == "?") {
    handleStatusQuery();
  } else if (cmd == "M17") {
    enableMotors(true);
    Serial.println("OK");
  } else if (cmd == "M18") {
    enableMotors(false);
    Serial.println("OK");
  } else if (cmd == "RESET") {
    emergencyStop = false;
    enableMotors(true);
    Serial.println("OK");
  } else if (cmd == "POS") {
    // Report current position
    Serial.print("A:");
    Serial.print(currentAngle1);
    Serial.print(" B:");
    Serial.print(currentAngle2);
    Serial.print(" C:");
    Serial.println(currentAngle3);
  } else {
    Serial.println("ERROR:Unknown command");
  }
}

void handleMoveCommand(String cmd) {
  // Parse: M1 A<angle1> B<angle2> C<angle3>
  float a1 = 0, a2 = 0, a3 = 0;

  int aIdx = cmd.indexOf('A');
  int bIdx = cmd.indexOf('B');
  int cIdx = cmd.indexOf('C');

  if (aIdx == -1 || bIdx == -1 || cIdx == -1) {
    Serial.println("ERROR:Invalid format. Use: M1 A# B# C#");
    return;
  }

  a1 = cmd.substring(aIdx + 1, bIdx).toFloat();
  a2 = cmd.substring(bIdx + 1, cIdx).toFloat();
  a3 = cmd.substring(cIdx + 1).toFloat();

  if (emergencyStop) {
    Serial.println("ERROR:E-Stop active. Send RESET");
    return;
  }

  // Convert angles to steps
  long steps1 = (long)(a1 * STEPS_PER_DEGREE);
  long steps2 = (long)(a2 * STEPS_PER_DEGREE);
  long steps3 = (long)(a3 * STEPS_PER_DEGREE);

  // Set absolute target positions
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  motor3.moveTo(steps3);

  // Block until motion complete (motors run in parallel)
  while (motor1.isRunning() || motor2.isRunning() || motor3.isRunning()) {
    motor1.run();
    motor2.run();
    motor3.run();

    // Check for emergency stop during motion
    if (emergencyStop) {
      motor1.stop();
      motor2.stop();
      motor3.stop();
      Serial.println("ERROR:Motion aborted");
      return;
    }

    // Check for new serial commands (allows M0 during motion)
    if (Serial.available() > 0) {
      char c = Serial.peek();
      if (c == 'M' || c == 'm') {
        // Potential emergency stop, process it
        String emergencyCmd = Serial.readStringUntil('\n');
        emergencyCmd.trim();
        emergencyCmd.toUpperCase();
        if (emergencyCmd == "M0") {
          handleEmergencyStop();
          return;
        }
      }
    }
  }

  // Update tracked angles
  currentAngle1 = a1;
  currentAngle2 = a2;
  currentAngle3 = a3;

  Serial.println("OK");
}

void handleHomingCommand() {
  if (emergencyStop) {
    Serial.println("ERROR:E-Stop active");
    return;
  }

  Serial.println("Homing...");

  // Use slower speed for homing
  motor1.setMaxSpeed(HOMING_SPEED);
  motor2.setMaxSpeed(HOMING_SPEED);
  motor3.setMaxSpeed(HOMING_SPEED);

  // Home each axis sequentially for safety
  bool success = true;

  if (success)
    success = homeAxis(motor1, M1_ENDSTOP_PIN, M1_HOME_DIR, "M1");
  if (success)
    success = homeAxis(motor2, M2_ENDSTOP_PIN, M2_HOME_DIR, "M2");
  if (success)
    success = homeAxis(motor3, M3_ENDSTOP_PIN, M3_HOME_DIR, "M3");

  // Restore normal speed
  motor1.setMaxSpeed(MAX_SPEED);
  motor2.setMaxSpeed(MAX_SPEED);
  motor3.setMaxSpeed(MAX_SPEED);

  if (success) {
    isHomed = true;
    currentAngle1 = 0;
    currentAngle2 = 0;
    currentAngle3 = 0;
    Serial.println("OK");
  } else {
    Serial.println("ERROR:Homing failed");
  }
}

bool homeAxis(AccelStepper &motor, int endstopPin, int homeDir, String name) {
  unsigned long startTime = millis();
  unsigned long timeout = 60000; // 60 second timeout

  // Move toward endstop
  motor.move(homeDir * 100000); // Large move in homing direction

  // Move until endstop triggered or timeout
  while (!isEndstopTriggered(endstopPin)) {
    motor.run();

    if (emergencyStop) {
      motor.stop();
      return false;
    }

    // Timeout check
    if (millis() - startTime > timeout) {
      motor.stop();
      Serial.print("ERROR:");
      Serial.print(name);
      Serial.println(" homing timeout - Check wiring/switch or motor power");
      return false;
    }

    // Travel limit check (if motor thinks it finished moving but no switch hit)
    if (motor.distanceToGo() == 0) {
      motor.stop();
      Serial.print("ERROR:");
      Serial.print(name);
      Serial.println(
          " travel limit reached - Switch not hit or wrong direction");
      return false;
    }
  }

  // Stop and set as zero
  motor.stop();
  delay(100); // Brief settle time for A4988
  motor.setCurrentPosition(0);

  // Back off from endstop
  motor.move(-homeDir * HOMING_BACKOFF);
  while (motor.isRunning()) {
    motor.run();
  }

  // Final zero position
  motor.setCurrentPosition(0);

  Serial.print(name);
  Serial.println(" homed");

  return true;
}

void handleEmergencyStop() {
  emergencyStop = true;

  // Immediate stop all motors
  motor1.stop();
  motor2.stop();
  motor3.stop();

  // Disable drivers (cuts power to motors)
  enableMotors(false);

  Serial.println("OK");
}

void handleStatusQuery() {
  int e1 = isEndstopTriggered(M1_ENDSTOP_PIN) ? 1 : 0;
  int e2 = isEndstopTriggered(M2_ENDSTOP_PIN) ? 1 : 0;
  int e3 = isEndstopTriggered(M3_ENDSTOP_PIN) ? 1 : 0;

  Serial.print("E1:");
  Serial.print(e1);
  Serial.print(" E2:");
  Serial.print(e2);
  Serial.print(" E3:");
  Serial.print(e3);
  Serial.print(" H:");
  Serial.println(isHomed ? "1" : "0");
}

// ============================================
// Helper Functions
// ============================================

bool isEndstopTriggered(int pin) {
  return digitalRead(pin) == ENDSTOP_TRIGGERED;
}

void enableMotors(bool enable) {
  // A4988 ENABLE is active LOW
  // LOW = motors enabled (holding torque)
  // HIGH = motors disabled (can free-spin)
  int state = enable ? LOW : HIGH;

  digitalWrite(M1_ENABLE_PIN, state);
  digitalWrite(M2_ENABLE_PIN, state);
  digitalWrite(M3_ENABLE_PIN, state);
}
