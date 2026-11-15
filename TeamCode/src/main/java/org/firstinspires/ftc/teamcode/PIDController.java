package org.firstinspires.ftc.teamcode;

public class PIDController {

    double Kp;
    double Ki;
    double Kd;

    double previousError = 0;

    double integral = 0.0;

    double sampleTime = 1.0;

    double target = 0.0;

    double output = 0.0;

    double deadbandSizeCoef = 0.7;

    double deadbandDepthCoef = 0.9;

    // Limit on the absolute value of the integral term (effectively unlimited by default).
    double integralLimit = 1e9;

    // When false, dt is always 1.0 (matches gains tuned with sampleTime = 1.0).
    // When true, dt is the measured time between loop() calls.
    boolean useRealTime = false;

    // Timestamp of the previous loop() call in nanoseconds; 0 means unset.
    long lastNanos = 0;

    PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.target = 0;
    }

    void setCoefs(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    void setTarget(double target) {
        this.target = target;
    }

    void setIntegralLimit(double limit) {
        this.integralLimit = limit;
    }

    void setUseRealTime(boolean useRealTime) {
        this.useRealTime = useRealTime;
    }

    void reset() {
        integral = 0.0;
        previousError = 0.0;
        output = 0.0;
        lastNanos = 0;
    }

    double loop(double value) {
        double dt = computeDt();
        sampleTime = dt;

        double error = calculateDeadband(this.target - value);
        double proportional = error;

        integral += error * dt;
        integral = Math.min(Math.max(integral, -integralLimit), integralLimit);

        double derivative = (error - previousError) / dt;

        this.output = Kp * proportional + Ki * integral + Kd * derivative;

        previousError = error;

        return output;
    }

    // Returns the time step for this call. Always records the current time.
    private double computeDt() {
        long now = System.nanoTime();
        double measured = (lastNanos == 0) ? -1.0 : (now - lastNanos) / 1e9;
        lastNanos = now;

        if (!useRealTime) {
            return 1.0;
        }
        if (measured <= 0 || measured > 0.5) {
            return 0.02;
        }
        return measured;
    }

    public void setDeadbandStuff(double deadbandSizeCoef, double deadbandDepthCoef) {
        this.deadbandSizeCoef = deadbandSizeCoef;
        this.deadbandDepthCoef = deadbandDepthCoef;
    }

    double calculateDeadband(double v) {
        double x = this.deadbandSizeCoef * v;
        return v * (1 - (4 * this.deadbandDepthCoef) * (1/(1+Math.pow(Math.E, x))) * (1-1/(1+Math.pow(Math.E, x))));
    }
}
