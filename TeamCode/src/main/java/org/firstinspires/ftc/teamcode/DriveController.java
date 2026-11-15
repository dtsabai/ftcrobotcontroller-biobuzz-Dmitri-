package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveController {

    // TODO: measure this on the robot (max ticks/sec at full power, e.g. read getVelocity() while
    // driving flat out) and set it just under that. 2000 is a conservative placeholder.
    static final double MAX_TICKS_PER_SEC = 2000.0;

    // Minimum wheel power (0..1) used only by driveHold(): below this the wheels stall on friction.
    double minHoldPower = 0.0;

    // Max change in wheel power per second (0 = disabled). Limits wheel slip on big step inputs.
    double maxAccelPerSec = 0.0;

    // Time source in nanoseconds; replaceable so the slew limit can be tested in simulation.
    java.util.function.LongSupplier clock = System::nanoTime;

    private final double[] lastWheelPower = new double[4];
    private long lastDriveNanos = 0;

    public PIDController drivingPID;// = new PIDController(1.0, 0.1, 0.3); // meh pid

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    IMU imu;

    Orientation angles;

    double measuredYaw;

    double yawZero;

    DriveController(IMU imu, DcMotor frontRightDrive, DcMotor frontLeftDrive, DcMotor backLeftDrive, DcMotor backRightDrive, PIDController pid) {
        this.drivingPID = pid; // meh pid
        this.imu = imu;
        this.frontRightDrive = frontRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.measuredYaw = 0.0;
    }

    void init() {
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void stop() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void setMeasuredYaw(double yaw) {
        this.measuredYaw = yaw;
    }

    public void driveFieldRelativeAuto(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // TODO: Continuousalize this, or use quaternion maybe, because I think wrapping around is causing the issue that only appears half the time
//        angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double yaw_corrected = ((((this.measuredYaw) - this.yawZero) + 180.0)) % 360.0 - 180.0;

        double currentError = AngleUnit.normalizeRadians(rotate - yaw_corrected * (Math.PI/180));
        drivingPID.setTarget(0);
        rotate = drivingPID.loop(currentError);

        theta = AngleUnit.normalizeRadians(theta - (yaw_corrected) * (3.141592653589/180));//AngleUnit.n

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // TODO: Continuousalize this, or use quaternion maybe, because I think wrapping around is causing the issue that only appears half the time
//        angles = this.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double yaw_corrected = ((((this.measuredYaw) - this.yawZero) + 180.0)) % 360.0 - 180.0;

//        double currentError = AngleUnit.normalizeRadians(rotate - yaw_corrected * (Math.PI/180));
//        drivingPID.setTarget(0);
//        rotate = drivingPID.loop(currentError);

        theta = AngleUnit.normalizeRadians(theta - (yaw_corrected) * (3.141592653589/180));//AngleUnit.n

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Like driveFieldRelative(), but boosts a small non-zero command up to minHoldPower so the robot
    // keeps creeping into the target instead of stalling on friction. Used by position-hold code.
    public void driveHoldFieldRelative(double forward, double right, double rotate) {
        double mag = Math.max(Math.hypot(forward, right), Math.abs(rotate));
        if (mag > 1e-6 && mag < minHoldPower) {
            double k = minHoldPower / mag;
            forward *= k;
            right *= k;
            rotate *= k;
        }
        driveFieldRelative(forward, right, rotate);
    }

    // Moves a wheel power toward its target by at most maxAccelPerSec * dt.
    private double slew(int i, double target, double dt) {
        if (maxAccelPerSec <= 0 || dt <= 0) {
            lastWheelPower[i] = target;
            return target;
        }
        double step = maxAccelPerSec * dt;
        double p = lastWheelPower[i] + Math.max(-step, Math.min(step, target - lastWheelPower[i]));
        lastWheelPower[i] = p;
        return p;
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // Velocity control: each wheel runs at a fraction of MAX_TICKS_PER_SEC. Wheel powers are
        // scaled by maxPower (>= 1.0) so no wheel exceeds 100% and the ratios stay correct.
        // maxSpeed can be lowered for outreaches.
        // Motors are kept in RUN_USING_ENCODER (set in init()), so no mode switching is needed here.
        long now = clock.getAsLong();
        double dt = lastDriveNanos == 0 ? 0 : Math.min((now - lastDriveNanos) / 1e9, 0.1);
        lastDriveNanos = now;

        double fl = slew(0, frontLeftPower / maxPower, dt);
        double fr = slew(1, frontRightPower / maxPower, dt);
        double bl = slew(2, backLeftPower / maxPower, dt);
        double br = slew(3, backRightPower / maxPower, dt);

        ((DcMotorEx) frontLeftDrive).setVelocity(MAX_TICKS_PER_SEC * maxSpeed * fl);
        ((DcMotorEx) frontRightDrive).setVelocity(MAX_TICKS_PER_SEC * maxSpeed * fr);
        ((DcMotorEx) backLeftDrive).setVelocity(MAX_TICKS_PER_SEC * maxSpeed * bl);
        ((DcMotorEx) backRightDrive).setVelocity(MAX_TICKS_PER_SEC * maxSpeed * br);
    }
}
