/*
 * Simple Motor Sweep Test
 * =======================
 * AUTOMATIC TEST - No Serial Commands Needed
 *
 * 1. Disconnects all endstops (Safety).
 * 2. Rotates Motor 1, then Motor 2, then Motor 3.
 * 3. Direction: Forward 1 revolution, pause, Backward 1 revolution.
 *
 * REQUIREMENTS:
 * - 12V Power Supply connected to RAMPS (Green Connector).
 * - USB Connected for code upload.
 */

// PINS
#define M1_STEP 54
#define M1_DIR 55
#define M1_EN 38

#define M2_STEP 60
#define M2_DIR 61
#define M2_EN 56

#define M3_STEP 46
#define M3_DIR 48
#define M3_EN 62

#define LED_PIN 13

// Settings
const int stepDelay = 1000; // Speed (Lower = Faster). 1000us = 1ms/step
const int revSteps =
    3200; // Steps for one move (Assuming 1/16 microstepping, 200*16 = 3200/rev)
          // If drivers are 1/1 (no jumpers), this will be 16 revolutions!
          // If drivers are 1/16 (all jumpers), this is 1 revolution.

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Setup Pins
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  pinMode(M3_STEP, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT);

  // Enable Motors (Active LOW)
  digitalWrite(M1_EN, LOW);
  digitalWrite(M2_EN, LOW);
  digitalWrite(M3_EN, LOW);

  digitalWrite(LED_PIN, HIGH); // LED ON
}

void loop() {
  // === Test Motor 1 ===
  runMotor(M1_STEP, M1_DIR, "Motor 1");
  delay(500);

  // === Test Motor 2 ===
  runMotor(M2_STEP, M2_DIR, "Motor 2");
  delay(500);

  // === Test Motor 3 ===
  runMotor(M3_STEP, M3_DIR, "Motor 3");
  delay(2000); // Wait before repeating
}

void runMotor(int stepPin, int dirPin, String name) {
  // Move one way
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < revSteps / 4;
       i++) { // Move 1/4 of a full rotation to be safe
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  delay(200);

  // Move back
  digitalWrite(dirPin, LOW);
  for (int i = 0; i < revSteps / 4; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}
