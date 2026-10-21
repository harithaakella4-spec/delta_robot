# Troubleshooting: Motors Not Moving (Silent)

If the motors are completely silent (no buzzing, no vibration) while running `motor_check.ino`, it is almost certainly a **Power** or **Driver** issue.

## 1. Check Power Supply (The #1 Cause)
The USB cable **CANNOT** power the motors. You must have a robust 12V power supply.

*   **RAMPS Power Connector (Green Block)**:
    *   There are 4 holes: `11A`, `11A`, `5A`, `5A` (or labelled `>11A` and `5A`).
    *   **The Motors run from the 5A rail.**
    *   **Verify:** Ensure your 12V wires are securely screwed into the **Bottom Pair** (usually labelled 5A). Better yet, power **BOTH** pairs if you can, but the 5A one is critical for motors.
    *   **Check Polarity:** `+` and `-` must be correct.

## 2. Check the Fuse (Yellow Components)
You mentioned a "Power Surge" earlier. This might have tripped the **Polyfuses** on the RAMPS board (the big flat yellow things near the power connector).
*   **Test:** Touch them carefully. Are they extremely hot? If so, they tripped. Unplug everything and wait 10 mins for them to cool and reset.
*   **Damage:** If there was a hard 5V-GND short, the Arduino voltage regulator might be dead, but if `hardware_test` code uploads and runs (LED blinks?), the Arduino logic is likely fine.

## 3. Check Driver Orientation (Critical)
If A4988 drivers are plugged in backwards, they burn instantly.
*   **Correct Orientation:** The **trimpot (metal screw)** on the A4988 should face **AWAY** from the power connector / heavy wiring side.
*   *Alternatively:* Look for the `EN` (Enable) pin label on the driver and match it to `EN` on RAMPS.

## 4. Check Motor Wiring
*   The RAMPS headers are `2B 2A 1A 1B`.
*   Standard Stepper Cables (4 wires): Blue-Red-Green-Black (or similar).
*   **Test:** If you have a multimeter, check continuity.
    *   Pair 1: Pins 1 & 2 on the connector should beep.
    *   Pair 2: Pins 3 & 4 on the connector should beep.
    *   If they are mismatched, the motor won't turn (but it usually usually buzzes/vibrates, not silent).

## 5. Current Limit (Vref)
If the little screw on the Driver is set to 0, the motor gets 0 amps.
*   You need a screwdriver.
*   With everything ON, gently turn the screw clockwise a tiny bit.
*   **Warning:** Monitor temperature.

## Summary Checklist
1. [ ] 12V Power connected to the 5A input?
2. [ ] 12V Power switch turned ON?
3. [ ] Polyfuses cool to touch?
4. [ ] Driver trimpot facing away from power block?
