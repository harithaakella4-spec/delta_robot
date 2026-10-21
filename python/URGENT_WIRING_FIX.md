# CRITICAL HARDWARE ALERT: USB Power Surge

> [!CAUTION] 
> **STOP IMMEDIATELY.** Do not press the switches again until you verify the wiring. The "Power Surge" warning means you are shorting 5V directly to Ground. This can permanently damage your computer's USB port or the Arduino.

## The Cause: Incorrect Endstop Wiring

On the RAMPS 1.4 shield, the endstop headers have 3 pins:
1.  **S (Signal)** - Top / Inner
2.  **- (Ground)** - Middle
3.  **+ (VCC / 5V)** - Bottom / Outer

**The Issue:** Your switch is likely connecting the **+ (5V)** pin to the **- (Ground)** pin when pressed. This creates a "Short Circuit".

## How to Fix It

### If you are using simple 2-pin mechanical switches (Recommended)
You only need to connect TWO wires.
*   **Wire 1**: Connect to **S (Signal)**
*   **Wire 2**: Connect to **- (Ground)**
*   **Leave the + (VCC) pin DISCONNECTED.**

Do not use the 3rd pin on the RAMPS board.

### If you are using a 3-pin Endstop Board (PCB with LED)
*   You must be extremely careful with the wire order. Colors are not standard!
*   Verify the markings on the Endstop PCB (`S`, `G`, `V`) match the markings on the RAMPS board.
*   **Common Mistake**: Plugging the connector in backwards or shifted, which sends GND to VCC.

## Testing after Fixing Routing
1.  Unplug USB.
2.  Correct the wiring (Ensure VCC is not involved for mechanical switches).
3.  Plug USB back in.
4.  Run the `hardware_test.ino` sketch.
5.  Type `E` to monitor status.
6.  Gently click the switch. **If the LED on the Arduino goes dark or the PC plays a disconnect sound, UNPLUG IMMEDIATELY.**

## Motor Issues
The motors likely weren't working because the power supply was shorting out or the protection circuit was tripping. Fix the endstops first.
Once the short is fixed:
1.  Make sure the 12V power supply is connected to the RAMPS (green connector).
2.  **USB does not power the motors.** You need the 12V brick.
