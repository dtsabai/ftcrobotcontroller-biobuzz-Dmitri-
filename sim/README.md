# Drive simulation

Runs the real `DriveController` and `PIDController` against stub FTC classes and a simple
motor/robot model (first-order wheel response, friction deadband, 1-loop sensor latency).
Not part of the Gradle build.

    sim/run.sh Sim     # waypoint results for each config
    sim/run.sh Sweep   # acceleration-limit sweep
    sim/run.sh Dead    # PID deadband x minHoldPower x friction

**Assumptions (not measured):** 4 in wheels, 537 ticks/rev, 0.08 s motor time constant,
2800 ticks/s true max, 50 Hz loop. Controller sign conventions are taken as already correct
(they match the working teleop), so this checks tuning/settling, not wiring or motor directions.
It does not model wheel slip, field walls, game elements or other robots.

**Findings**
- The default PID deadband leaves 1-3 in of steady-state error; setting depth 0 on the OTOS x/y
  controllers cuts it 3-10x.
- `minHoldPower > 0` makes hold mode creep forever (1-2 in/s) instead of settling: keep it 0.
- The acceleration limit does not change simulated results (no slip modelled); check on the robot.
