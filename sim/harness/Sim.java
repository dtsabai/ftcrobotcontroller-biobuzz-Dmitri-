package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Sim {
    // simulated motor: first-order velocity response, stiction deadband, true max speed
    static class SimMotor implements DcMotorEx {
        double cmd = 0, vel = 0;
        public void setVelocity(double t) { cmd = t; }
        public void setDirection(Direction d) {} public void setMode(RunMode m) {}
        public void setZeroPowerBehavior(ZeroPowerBehavior z) {} public void setPower(double p) { cmd = 0; }
        void step(double dt, double tau, double stiction, double trueMax) {
            double c = Math.max(-trueMax, Math.min(trueMax, cmd));
            if (Math.abs(c) < stiction) c = 0;               // too weak to overcome friction
            vel += (c - vel) * (1 - Math.exp(-dt / tau));
        }
    }
    static double deadDepth = 0.9; // PIDController default
    static double lastSpeedTail;
    static final double K = 12.566 / 537.0; // inches per tick (4in wheel, 537 ticks/rev) - ASSUMPTION
    static final double ROT_R = 10.0;       // inches: effective turning radius - ASSUMPTION

    static class Result { double t; boolean reached; double maxOver, finalErr, peakAccel; }

    static Result run(double tx, double ty, double th, double minHold, double accel, double stiction, boolean oneStage, long[] stats) {
        SimMotor fl = new SimMotor(), fr = new SimMotor(), bl = new SimMotor(), br = new SimMotor();
        DriveController dc = new DriveController(null, fr, fl, bl, br, new PIDController(0.02, 0.001, 0.001));
        final long[] clk = {1}; dc.clock = () -> clk[0]; dc.maxAccelPerSec = accel; dc.minHoldPower = minHold;
        PIDController px = new PIDController(0.08, 0.00003, 0.125), py = new PIDController(0.08, 0.00003, 0.125), ph = new PIDController(0.01, 0.00001, 0.009);
        px.setDeadbandStuff(0.7, deadDepth); py.setDeadbandStuff(0.7, deadDepth); px.setTarget(tx); py.setTarget(ty); ph.setTarget(0);
        double x = 0, y = 0, h = 0, dt = 0.02, prevV = 0; double tailMax = 0; double mx = 0, my = 0, mh = 0;
        Result r = new Result(); double reachedT = -1; double maxDist = 0;
        double lastX = x, lastY = y, lastH = h; // 1-loop sensor latency
        for (int i = 0; i < 500; i++) {
            double t = i * dt; clk[0] = 1 + (long) (t * 1e9);
            dc.setMeasuredYaw(lastH);
            double posErr = Math.hypot(tx - lastX, ty - lastY), hErr = AngleUnit.normalizeDegrees(th - lastH);
            if (reachedT < 0 && posErr < 1.0 && Math.abs(hErr) < 3.0) { reachedT = t; }
            double xs = px.loop(lastX), ys = py.loop(lastY), hs = ph.loop(AngleUnit.normalizeDegrees(lastH - th));
            dc.driveHoldFieldRelative(-ys, -xs, hs);
            for (SimMotor m : new SimMotor[]{fl, fr, bl, br}) m.step(dt, 0.08, stiction, 2800);
            // inverse mecanum mapping (matches DriveController.drive wheel equations)
            double f = (fl.vel + fr.vel + bl.vel + br.vel) / 4, rt = (fl.vel - fr.vel + br.vel - bl.vel) / 4, rot = (fl.vel - fr.vel - br.vel + bl.vel) / 4;
            double yawR = Math.toRadians(h);
            // robot->field; sign chosen so the controller feedback is negative (matches real robot, see notes)
            double fxCmd = rt * Math.cos(yawR) - f * Math.sin(yawR), fyCmd = rt * Math.sin(yawR) + f * Math.cos(yawR);
            double vx = -fxCmd * K, vy = -fyCmd * K, w = Math.toDegrees(rot * K / ROT_R);
            double speed = Math.hypot(vx, vy); r.peakAccel = Math.max(r.peakAccel, Math.abs(speed - prevV) / dt); prevV = speed;
            lastX = x; lastY = y; lastH = h;
            x += vx * dt; y += vy * dt; h += w * dt;
            if (i >= 400) tailMax = Math.max(tailMax, speed);
            maxDist = Math.max(maxDist, Math.hypot(x, y));
            double along = (x * tx + y * ty) / Math.max(1e-9, Math.hypot(tx, ty));
            r.maxOver = Math.max(r.maxOver, along - Math.hypot(tx, ty));
        }
        r.t = reachedT; r.reached = reachedT >= 0; r.finalErr = Math.hypot(tx - x, ty - y);
        lastSpeedTail = tailMax;
        // jitter: wheel command motion in last 2 s
        return r;
    }
    public static void main(String[] a) {
        double[][] wp = {{24,0,0},{24,24,0},{0,0,0},{60,40,90},{3,0,0}};
        double[] stictions = {0, 60, 150};
        for (double st : stictions) {
            System.out.printf("--- stiction deadband %.0f ticks/s ---%n", st);
            System.out.printf("%-14s %-22s %7s %8s %9s %12s%n", "waypoint", "config", "reached", "time(s)", "overshoot", "final err(in)");
            for (double[] w : wp) {
                String[] names = {"baseline", "minHold .08", "accel 6", "minHold+accel"};
                double[][] cfg = {{0,0},{0.08,0},{0,6},{0.08,6}};
                for (int c = 0; c < 4; c++) {
                    Result r = run(w[0], w[1], w[2], cfg[c][0], cfg[c][1], st, false, null);
                    System.out.printf("%-14s %-22s %7s %8.2f %9.2f %12.2f%n", String.format("(%.0f,%.0f,%.0f)", w[0], w[1], w[2]), names[c], r.reached, r.t, Math.max(0, r.maxOver), r.finalErr);
                }
            }
        }
    }
}
