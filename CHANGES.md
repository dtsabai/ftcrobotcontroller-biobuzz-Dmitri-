# What changed and why

Summary of the code audit and fixes on branch `claude/wonderful-fermat-bwmmnl`.
All code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` unless noted.

**Status: nothing here has been built in Android Studio or run on the robot.**
See "Before you use it" at the bottom.

## Bugs fixed

| File | Problem | Fix |
|---|---|---|
| `SVPUtiliserCeModeTeleop` | The AprilTag camera processor was stored in a local variable instead of the class field, so the field stayed empty and the first loop crashed. | Assign the field. |
| `SVPUtiliserCeModeTeleop` | If the program crashed, the web servers (ports 8885/8886) and camera were never closed, so the next run failed with "address already in use". | Close everything in a `try/finally`. |
| `DriveController` | Back-right motor never got its tuning: the back-left line was written twice. | Second line now targets back-right. |
| `WebInterface`, `WebTelemetryStreamer` | After `stop()`, both server loops kept spinning forever and leaked a thread per run. The telemetry queue also grew without limit when no browser was connected. | Loops exit on stop, client socket is closed, queue is capped and frames are dropped when nobody is listening. |
| `WebInterface` | Parameter map was not thread-safe; a mistyped parameter name returned null and crashed; `/setValue` accepted anything. | Thread-safe map, missing names return 0.0, `/setValue` rejects unknown names and bad numbers and clamps to the slider's min/max. |

## Performance and robustness

- **Hub bulk caching** is on, so the robot reads sensors in one batch instead of one request per device.
- **Shooter tuning is only sent when a value changes**, not every loop.
- **Web telemetry is sent every 3rd loop**, not every loop.
- **Camera and drive controller are set up before Start**, so no seconds are lost after the match begins.
- **`PIDController`** gained `reset()` (clears built-up error), an optional real-time time step, and an integral limit. Defaults behave exactly as before, so existing tuning is unchanged.
- **OTOS hold (hold X)** now resets its controllers when you press X, so it starts clean. Resetting the OTOS also resets the heading offset.

## Behavior changes (these change how the robot drives)

1. **Drive uses velocity control.** `drive()` used to set power to 1.0 and keep moving a target position each loop, so speed depended on loop timing. It now sets each wheel's speed directly (`setVelocity`).
   `MAX_TICKS_PER_SEC = 2000` in `DriveController` is a **placeholder**. Measure your real top speed and set it just below that.
2. **Turret moved to gamepad2's right stick (Y).** Before, it shared gamepad1's right stick with rotation, so turning the robot also moved the turret. You now need gamepad2 connected to aim.
3. **PID deadband turned off for the OTOS X/Y controllers.** The default deadband ignored small errors and left the robot 1.5-3 in from its target. With it off, the simulation shows 3-10x less error. The heading controller's deadband is unchanged and untested.
4. **Optional acceleration limit** (`maxAccelPerSec`) is on at 6 per second in hold mode and the autonomous; joystick driving is unaffected. The simulation showed no effect, so it is unproven. Set it to 0 if hold mode feels sluggish.
5. **Optional minimum creep power** (`minHoldPower`) exists but is set to 0. In the simulation any value above 0 made the robot creep forever instead of stopping.

## New and removed files

- **Added `AutoOtosDrive.java`** - a simple autonomous that drives to a list of waypoints using the OTOS sensor. The waypoints (24,0), (24,24), (0,0) and the gains are **placeholders**.
- **Added `sim/`** - a small computer model of the drive (see `sim/README.md`). Run `sim/run.sh Sim`. It is not part of the Gradle build.
- **Removed `BasicAutoClass.java` and `AutoStateThing.java`** - no OpMode used them, and `BasicAutoClass` looked for hardware this robot doesn't have (`external_imu`, `shooter`, `feeder2`). They are still in git history.
- **Cleanup** - about 390 lines of dead and commented-out code removed from the teleop, a few unused imports removed, magic numbers named (`TICKS_PER_REV`, `VELOCITY_GRAPH_SCALE`), and the on-screen help text now matches the real controls.

## Not changed

- **The shooter still starts ON when you press Start** (`aToggler.currentState = true`). A comment marks the line; set it to `false` for a safer default.
- Mixed units for the intake/feeder/shooter speeds (some in RPM, some in ticks per second) were not unified.

## Before you use it

1. **Build** the project in Android Studio and fix any compile errors. Also confirm `AprilTagGameDatabase.getBioBuzzTagLibrary()` exists in your SDK version.
2. **Test with the wheels off the ground.** Check every wheel spins the right way, since the drive control mode changed.
3. **Set `MAX_TICKS_PER_SEC`** from a real measurement.
4. **Check the hold mode and autonomous** on the field and adjust the waypoints and gains.

## Simulation notes

The simulation runs the real `DriveController` and `PIDController` against a simple motor model. It assumes 4 in wheels, 537 ticks/rev, a 0.08 s motor response and a 50 Hz loop (all guesses). It checks settling and tuning, not wiring, wheel slip, the field, or game rules.
