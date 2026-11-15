package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto OTOS Drive", group = "Robot")
public class AutoOtosDrive extends LinearOpMode {

    // Waypoints as {x inches, y inches, heading degrees}
    private static final double[][] WAYPOINTS = {
            {24, 0, 0},
            {24, 24, 0},
            {0, 0, 0}
    };

    private static final double POS_TOLERANCE_IN = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 3.0;
    private static final double WAYPOINT_TIMEOUT_S = 5.0;

    // Default gains, same as SVPUtiliserCeModeTeleop
    private static final double KP_OTOS = 0.08, KI_OTOS = 0.00003, KD_OTOS = 0.125;
    private static final double KP_OTOSH = 0.01, KI_OTOSH = 0.00001, KD_OTOSH = 0.009;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private IMU imu;
    private SparkFunOTOS myOtos;
    private DriveController driveController;

    private PIDController xController;
    private PIDController yController;
    private PIDController hController;

    @Override
    public void runOpMode() {
        // Hardware and OTOS setup, same as the teleop
        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos0");
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.setOffset(new SparkFunOTOS.Pose2D(-6, 0, 90));

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive,
                backRightDrive, new PIDController(0.02, 0.001, 0.001));
        driveController.init();

        telemetry.addLine("Initialized. Press play to run waypoints.");
        telemetry.update();
        waitForStart();

        try {
            for (int i = 0; i < WAYPOINTS.length && opModeIsActive(); i++) {
                driveToWaypoint(i);
            }
            stopDrive();
        } finally {
            stopDrive();
        }
    }

    private void driveToWaypoint(int index) {
        double tx = WAYPOINTS[index][0];
        double ty = WAYPOINTS[index][1];
        double th = WAYPOINTS[index][2];

        // Fresh controllers so integral and derivative state does not carry between waypoints
        xController = new PIDController(KP_OTOS, KI_OTOS, KD_OTOS);
        yController = new PIDController(KP_OTOS, KI_OTOS, KD_OTOS);
        hController = new PIDController(KP_OTOSH, KI_OTOSH, KD_OTOSH);
        xController.setTarget(tx);
        yController.setTarget(ty);
        // Heading is passed in as a wrapped error relative to the target, so the target is 0
        hController.setTarget(0);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pose = myOtos.getPosition();
            driveController.setMeasuredYaw(pose.h);

            double posErr = Math.hypot(tx - pose.x, ty - pose.y);
            double headingErr = AngleUnit.normalizeDegrees(th - pose.h);
            boolean reached = posErr < POS_TOLERANCE_IN && Math.abs(headingErr) < HEADING_TOLERANCE_DEG;
            boolean timedOut = timer.seconds() > WAYPOINT_TIMEOUT_S;

            telemetry.addData("Waypoint", "%d of %d", index + 1, WAYPOINTS.length);
            telemetry.addData("Target", "x=%.1f y=%.1f h=%.1f", tx, ty, th);
            telemetry.addData("Pose", "x=%.2f y=%.2f h=%.1f", pose.x, pose.y, pose.h);
            telemetry.addData("Error", "pos=%.2f in heading=%.1f deg", posErr, headingErr);
            telemetry.addData("Status", reached ? "reached" : (timedOut ? "timeout" : "driving"));
            telemetry.update();

            if (reached || timedOut) {
                break;
            }

            double xSignal = xController.loop(pose.x);
            double ySignal = yController.loop(pose.y);
            double rotationSignal = hController.loop(AngleUnit.normalizeDegrees(pose.h - th));

            driveController.driveFieldRelative(-ySignal, -xSignal, rotationSignal);
        }
    }

    private void stopDrive() {
        driveController.drive(0, 0, 0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
