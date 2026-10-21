/*
 * RAMPS 1.4 Hardware Test Sketch
 * ==============================
 * This sketch helps diagnose hardware issues by isolating components.
 *
 * COMMANDS (Type in Serial Monitor at 115200 baud):
 * -------------------------------------------------
 * 'E' : Report Endstop Status (Pressed/Open)
 * '1' : Test Motor 1 (X Axis) - Spins 200 steps one way, then back
 * '2' : Test Motor 2 (Y Axis) - Spins 200 steps one way, then back
 * '3' : Test Motor 3 (Z Axis) - Spins 200 steps one way, then back
 * 'A' : Enable All Motors (Lock position)
 * 'D' : Disable All Motors (Free spin)
 */

// PINS FOR RAMPS 1.4
#define M1_STEP_PIN 54
#define M1_DIR_PIN 55
#define M1_ENABLE_PIN 38
#define M1_ENDSTOP_PIN 3

#define M2_STEP_PIN 60
#define M2_DIR_PIN 61
#define M2_ENABLE_PIN 56
#define M2_ENDSTOP_PIN 14

#define M3_STEP_PIN 46
#define M3_DIR_PIN 48
#define M3_ENABLE_PIN 62
#define M3_ENDSTOP_PIN 18

// MAX PINS (Alternative Ports)
#define X_MAX_PIN 2
#define Y_MAX_PIN 15
#define Z_MAX_PIN 19

#define LED_PIN 13

// Settings
int stepDelay = 1000; // Microseconds between steps (Lower = Faster)
bool invertDir = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Setup Motor Pins
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENABLE_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENABLE_PIN, OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M3_ENABLE_PIN, OUTPUT);

  // Setup Endstops (Min and Max)
  pinMode(M1_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(M2_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(M3_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);

  // Enable Motors (Active LOW)
  digitalWrite(M1_ENABLE_PIN, LOW);
  digitalWrite(M2_ENABLE_PIN, LOW);
  digitalWrite(M3_ENABLE_PIN, LOW);

  Serial.println("=== RAMPS 1.4 HARDWARE TEST ===");
  Serial.println("Send 'E' to check Endstops");
  Serial.println("Send '1', '2', or '3' to test Motors");
  Serial.println("Send 'D' to disable motors (unlock)");
  Serial.println("Send 'A' to enable motors (lock)");
  Serial.println("===============================");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    cmd = toupper(cmd);

    // Discard newlines
    if (cmd == '\n' || cmd == '\r')
      return;

    Serial.print("Command received: ");
    Serial.println(cmd);

    switch (cmd) {
    case 'E':
      checkEndstops();
      break;
    case '1':
      testMotor(M1_STEP_PIN, M1_DIR_PIN, "Motor 1 (X)");
      break;
    case '2':
      testMotor(M2_STEP_PIN, M2_DIR_PIN, "Motor 2 (Y)");
      break;
    case '3':
      testMotor(M3_STEP_PIN, M3_DIR_PIN, "Motor 3 (Z)");
      break;
    case 'D':
      digitalWrite(M1_ENABLE_PIN, HIGH);
      digitalWrite(M2_ENABLE_PIN, HIGH);
      digitalWrite(M3_ENABLE_PIN, HIGH);
      Serial.println("Motors DISABLED (Un-locked)");
      break;
    case 'A':
      digitalWrite(M1_ENABLE_PIN, LOW);
      digitalWrite(M2_ENABLE_PIN, LOW);
      digitalWrite(M3_ENABLE_PIN, LOW);
      Serial.println("Motors ENABLED (Locked)");
      break;
    default:
      Serial.println("Unknown command.");
      break;
    }
  }
}

void checkEndstops() {
  Serial.println("Reading Endstops for 5 seconds...");
  Serial.println("PRESS SWITCHES NOW");

  unsigned long start = millis();
  while (millis() - start < 5000) {
    bool e1 = digitalRead(M1_ENDSTOP_PIN);
    bool e2 = digitalRead(M2_ENDSTOP_PIN);
    bool e3 = digitalRead(M3_ENDSTOP_PIN);

    bool x_max = digitalRead(X_MAX_PIN);
    bool y_max = digitalRead(Y_MAX_PIN);
    bool z_max = digitalRead(Z_MAX_PIN);

    Serial.print("X_MIN:");
    Serial.print(e1);
    Serial.print(" Y_MIN:");
    Serial.print(e2);
    Serial.print(" Z_MIN:");
    Serial.print(e3);

    Serial.print(" | Z_MAX:");
    Serial.print(z_max); // Focus on Z_MAX
    Serial.println();

    delay(200);
  }
  Serial.println("Endstop check done.");
}

void testMotor(int stepPin, int dirPin, String name) {
  Serial.print("Testing ");
  Serial.print(name);
  Serial.println("...");

  // Forward
  Serial.println("  -> Moving Direction +");
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < 400; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  delay(500);

  // Backward
  Serial.println("  <- Moving Direction -");
  digitalWrite(dirPin, LOW);
  for (int i = 0; i < 400; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  Serial.println("Done.");
}
