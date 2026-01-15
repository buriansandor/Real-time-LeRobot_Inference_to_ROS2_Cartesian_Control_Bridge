# 📖 SO-100 Robotic Arm Setup and Calibration Guide
<!--Translated by Copilot-->

> This document describes how to prepare a freshly assembled (or Hugging Face LeRobot state) SO-100 robotic arm for precision Cartesian (XYZ) control.
## ⚠️ Why is this necessary?
The factory (or teleoperation) settings are prepared for "Master-Slave" copying, therefore:
1. They have narrow software limits (Safety Limits) that prevent reaching the mathematical home position (Candle).
2. Their 0 point is not vertical, but a convenience position (Raptor).

For Inverse Kinematics (IK), **unlimited range of motion** and **precise mathematical zero (Candle)** are required.

## 🛠️ Step 1: Removing Software Limits ("Unlock")
*Goal: Remove the "child lock" so the robot can reach the physical end positions.*
1. Connect the robot via USB (and the 12V power).
2. Run the limit-removing script (which we wrote earlier):
    ```Bash
    python unlock_motor_limits.py
    ```

   *(If you're afraid, use `backup_and_unlock.py` to save the factory values).*

3. Verification: On the console, you should see: Min=0, Max=4095.

## 📏 Step 2: The "Holy Candle" Calibration
*Goal: Teach the robot where the real, vertical zero point is.*

1. Run the manual calibrator:
   ```Bash
    python calibrate_manual_candle.py
   ```

2. The script turns off the motors (Torque OFF).
3. MANUALLY set the robot to the perfect Vertical (Candle) position:
   - Every arm should point straight up.
   - The wrist straight.
   - The waist (motor 1) should be exactly in the middle.
   - *(Tip: Use a spirit level or square if you can, but it should be fine by eye).*
4. Press **ENTER**.
5. The script saves the values to the `follower_calibration.csv` file.
   
   *`Example values: [2059, 2019, 1042, ...]`*

## 🧪 Step 3: Testing (Simple Move Test)
*Goal: Check if the robot can find positions on its own.*

1. Open simple_move_test.py.
2. IMPORTANT: Make sure that in the driver initialization, the calibration_file parameter points to the newly created CSV (or temporarily write the values into the code at the [FIX] section if there's a file handling issue).
3. Run the test:
   ```Bash
    python simple_move_test.py
   ```
4. Candle Test (Button: 4):
    - The robot should slowly but firmly rise to vertical.
    - If it goes to "Raptor" instead -> Wrong calibration file/values! (See step 2).
    - If it doesn't move at all -> Torque is not enabled! (The script must include writing to the `0x28` register).
5. Raptor Test (Button: 6):

        The robot should fold into the safety parking position.

## 🚑 Troubleshooting
- The robot doesn't move, just "hums":

        Probably the speed is too high, or the mechanics are stuck. Increase the move_time value (e.g., 5000 ms).

- The robot "died" (LED flashing, not responding):

        Overcurrent protection activated.

        Solution: Pull out the 12V AND the USB for 10 seconds.

- Goes to Raptor instead of Candle:

        The software is using the old (Hugging Face / Leader copied) calibration data. Force it to use the new, measured values.