# AUTONAV v2

Same robot, same wiring, **no extra hardware**. v2 adds manual driving, smarter
tag-based path planning, reliable obstacle handling with a detour that rejoins
the route, and live control of speed and the camera light.

```
backend/      Flask dashboard, camera + AprilTag pipeline, planner, robot link
firmware/     esp32_cam_stream.ino, esp32_devkit_motor_controller.ino
tools/        devkit_console.py, apriltag_standalone_debug.py, find_devkit_ip.py
tests/        firmware simulation, planner tests, backend tests
```

## Quick start

1. **Reflash both boards** (the DevKit firmware changed a lot; the CAM firmware
   gained a flash-level parameter). Libraries: *Adafruit VL53L0X*, ESP32 Arduino
   core 2.x (the same as v1).
2. `cd backend && pip install -r requirements.txt && python app.py`
3. Open <http://localhost:5000>.

Network settings are unchanged: hotspot `apex`, CAM `192.168.137.200`, DevKit
`192.168.137.201` (static IPs). The backend addresses now live in
**`backend/config.py`** instead of the top of `app.py`. If you change an IP in a
`.ino`, change it there too.

Wiring is exactly as before (motor driver on GPIO14/27/26/25/33/32, HC-SR04 A on
4/16, HC-SR04 B on 17/5, VL53L0X on I2C 21/22). Keep the 1k/2k divider on both
ECHO lines.

## What changed

### 1. Manual mode
Use the **Auto / Manual** switch. In Manual, hold the on-screen arrows or use
**W A S D / arrow keys** (forward + a side key curves; space stops). Releasing
stops the robot. Two dead-man protections: the backend stops the robot 0.6 s
after the browser goes quiet, and the firmware stops it if the backend goes
silent for 1.2 s.

The obstacle stop still works in Manual (forward only). A checkbox lets you turn
it off for calibration; it turns itself back on in Auto. There is no rear
sensor, so reversing is unprotected.

### 2. Better path planning (`backend/planner.py`, `apriltag_detector.py`)
- Weak detections and unknown IDs are ignored; a tag must be seen in 2
  consecutive frames.
- **Repeat-tag bug fixed:** v1 never re-armed after a tag left the view, so two
  identical direction tags in a row ignored the second one.
- Direction tags fire when the tag looks **big enough** (robot is close), not
  the instant it appears. The destination tag stops the robot the same way.
- While approaching a tag the robot **steers toward it** (differential drive).
- If the tag is on a wall the wall no longer counts as an obstacle: the planner
  acts on the tag ("tag lock").
- Lost for a long time (default 20 s with no tag): scan left/right, back up a
  little, scan again, then stop and report **Lost** instead of driving forever.
- If the camera stream dies mid-run, navigation stops.

### 3. Obstacle detection and detour (`esp32_devkit_motor_controller.ino`)
Why v1 misbehaved: the safety check only ran in the plain FORWARD state, a
no-echo reading counted as "clear", one bad sample decided everything, the
robot restarted blindly when the path cleared, and the backend never learned
what happened.

v2:
- Median-of-3 per sensor (a single noisy reading is ignored) and a hard
  "emergency" override for anything closer than 8 cm.
- Stop distance grows with speed; a slow zone reduces speed before the stop
  distance; stop/clear hysteresis stops flicker.
- Works for every forward motion, including steering commands.
- **Detour (sidestep and rejoin)**, run on the DevKit:

  `settle -> pick clearer side -> turn out -> sidestep -> turn back -> check
  -> pass -> turn in -> return the same distance -> realign -> resume`

  The sidesteps are added up as a signed lateral offset, and the return leg
  drives that same offset back, so the robot rejoins its original line. If the
  front is still blocked after a sidestep it repeats (up to 3 legs). Boxed in
  by a wall: it backs up once, then stops and reports `AVOID_FAIL`.
- The robot sends telemetry (left / centre / right distance, state) to the
  dashboard about 7 times a second, so you can see why it stopped.
- New: watchdog, and it now stops **before** a WiFi reconnect instead of after.

**Honest limits.** Without encoders or a gyro, turns and distances are timed
moves (about 10-20% error from battery level and floor grip). The detour is
therefore approximate; the next AprilTag re-centres the robot. It suits one
obstacle at a time. Ultrasonic sensors can miss soft, angled or thin objects;
the VL53L0X centre sensor covers some of that. The VL53L0X still uses the same
single-shot call as v1 (time-sliced with the two ultrasonics) because it is the
one API known to work with your library version.

### 4. Speed and camera light
- **Drive speed** (10-100%) and **turn speed** (20-100%) sliders. Timed turns
  are calibrated at the default turn speed; the firmware stretches or shortens
  them when you change it, but recalibrate if turns drift.
- **Camera light** slider (0-100%) uses the flash PWM on the ESP32-CAM
  (`/flash?level=0-255`). **Adjust automatically** tunes it from the average
  frame brightness with a dead-band, capped at level 200 to protect the supply
  (`AUTO_FLASH_MAX_LEVEL` in `config.py`). The flash draws a lot of current; if
  the CAM resets at high brightness, lower the cap or improve its 5V supply.

## Calibration (do this once)
Use the **Tuning** panel on the dashboard, or `python tools/devkit_console.py`.

1. **Turn length.** Manual mode, `LEFT 550` in the console. If it doesn't turn
   about 90 degrees, change **Turn length** until it does. Repeat for `RIGHT`.
2. **Sensors.** Watch the bars while holding a box in front of each sensor.
   Set **Stop distance** so the robot stops with room to spare. Angle the two
   ultrasonic sensors slightly outward and keep them clear of the chassis.
3. **Detour.** Put a box on the route. Adjust **Sidestep** and **Pass** so the
   robot clears the box's width and length, and **Detour speed** for smoothness.
4. **Tags.** Drive toward tags and read "Tag in view" (% of frame width). Set
   **Direction tag size** and **Destination tag size** a little below the size
   you see where you want the robot to act. Setting a ratio to 0 acts on first
   sight, like v1.

Settings changed in the panel last until the DevKit reboots. To keep them, copy
the values into the defaults in `struct Params` and `PlannerConfig` and reflash.

## Protocol (backend <-> DevKit, TCP 8080, newline-terminated)
Commands: `FORWARD RESUME BACKWARD LEFT[ ms] RIGHT[ ms] DRIVE l r STOP SPEED n
TURNSPEED n MODE AUTO|MANUAL SAFETY 0|1 AVOID 0|1 SET name value GET HB`.
The DevKit sends `T,...` telemetry, `E,NAME[,detail]` events (`OBSTACLE CLEAR
AVOID_START AVOID_DONE AVOID_FAIL WATCHDOG`) and `P,...` parameters. Full field
list is at the top of the DevKit sketch.

## Tests
```
python tests/test_planner.py
python tests/test_backend.py
g++ -std=c++17 -Itests/stubs tests/sim_devkit.cpp -o /tmp/sim && /tmp/sim
```
The last one compiles the real DevKit sketch on a PC against tiny Arduino
stand-ins and checks filtering, slow zone, the whole detour sequence, the
watchdog, manual mode and tuning. **These tests do not replace running on the
robot:** they cannot check the real sensors, motors, or the Adafruit and ESP32
libraries. Expect to do the calibration above.

## Not in v2 (needs hardware or is a bigger project)
Accurate heading or distance (needs an IMU or wheel encoders), a rear sensor,
mapping / multi-obstacle route planning.

## Network notes (unchanged from v1)
Static IPs need the hotspot subnet `192.168.137.x`. On another network, change
`STATIC_IP` / `GATEWAY` / `SUBNET` in both `.ino` files, reflash, and update
`backend/config.py`. mDNS names (`esp32cam.local`, `esp32devkit.local`) remain
as a backup but the backend uses the static IPs. The ESP32 can only join a
2.4 GHz network. Give the ESP32-CAM a proper 5V/2A supply.
