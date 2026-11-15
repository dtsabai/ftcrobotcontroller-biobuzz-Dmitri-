/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.ArrayList;
import java.util.Arrays;

import java.io.IOException;

import java.util.List;


/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive MAIN YAY", group = "Robot")
public class SVPUtiliserCeModeTeleop extends LinearOpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DcMotor shooterL;

    DcMotor intake;

    DcMotor feeder;

    DcMotor shooterR;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    SparkFunOTOS.Pose2D otos0Pos;

    WebTelemetryStreamer webTelemetryStreamer;

    WebInterface webInterface;

    Toggler aToggler = new Toggler();

    DriveController driveController;

    SparkFunOTOS myOtos;

    PIDController otosXController;
    PIDController otosYController;

    PIDController otosRotationController;
    PIDController apriltagRotationController;

    MotorSpeedTracker shooterL_spt;
    MotorSpeedTracker shooterR_spt;

    // Encoder ticks per motor revolution (28 for an ungeared REV HD Hex motor)
    private static final int TICKS_PER_REV = 28;
    // Empirical scale so velocity channels (deg/s / 6 = rpm) share a graph with the setpoints
    private static final double VELOCITY_GRAPH_SCALE = 20;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    Servo turret0;

    // last shooter PIDF values pushed to the motors (NaN until the first push)
    private double lastKpShooter = Double.NaN;
    private double lastKiShooter = Double.NaN;
    private double lastKdShooter = Double.NaN;
    private double lastKfShooter = Double.NaN;

    // gamepad1.x state from the previous loop, used for rising-edge detection
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        // let each REV hub cache bulk reads instead of issuing one transaction per device
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Rev9AxisImu.Parameters parameters = new Rev9AxisImu.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu"); // internal IMU
        imu.initialize(parameters);

        System.out.println("MAIN ACTUAL OPCODE YAY A");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos0");
        System.out.println("calibrating otos IMU");
        myOtos.calibrateImu();
        myOtos.resetTracking();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        myOtos.setOffset(new SparkFunOTOS.Pose2D(-6, 0, 90));


        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        turret0 = hardwareMap.get(Servo.class, "turret0");

        shooterL_spt = new MotorSpeedTracker(shooterL, 28);
        shooterR_spt = new MotorSpeedTracker(shooterR, 28);

        webTelemetryStreamer = new WebTelemetryStreamer(8886);
        webTelemetryStreamer.setChannels(new ArrayList<String>(Arrays.asList("shooterR_speed", "shooterR_current", "shooterL_speed", "shooterL_current", "intake_speed", "intake_current", "feeder_speed", "feeder_current", "SPshooter_speed", "SPfeeder_speed", "SPintake_speed", "battery_voltage", "otos0_x", "otos0_y", "otos0_h", "shooterL_speed_eec", "shooterR_speed_eec", "speed_backleft", "speed_backright", "speed_frontleft", "speed_frontright")));
        Thread webTelemetryStreamerThread = new Thread(webTelemetryStreamer);
        webTelemetryStreamerThread.start(); // start server

        webInterface = new WebInterface(8885);

        webInterface.addParameter("shooter_speed", 1850, 5000);
        webInterface.addParameter("feeder_speed", 2500, 4000);
        webInterface.addParameter("intake_speed", 2500, 5000);

        webInterface.addParameter("Kp_drive", 0.01);
        webInterface.addParameter("Ki_drive", 0.00);
        webInterface.addParameter("Kd_drive", 0.00);



        PIDFCoefficients defaults_shooter =((DcMotorEx) shooterL).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        defaults_shooter.p = 90; // set default

        webInterface.addParameter("Kp_shooter", defaults_shooter.p);
        webInterface.addParameter("Ki_shooter", defaults_shooter.i);
        webInterface.addParameter("Kd_shooter", defaults_shooter.d);
        webInterface.addParameter("Kf_shooter", defaults_shooter.f);


        // do not adjust these
        otosXController = new PIDController(0.0, 0.0, 0.0);
        otosYController = new PIDController(0.0, 0.0, 0.0);
        otosRotationController = new PIDController(0.0, 0.0, 0.0);
        // the default PID deadband swallows small errors and leaves 1-3 in of steady-state error
        otosXController.setDeadbandStuff(0.7, 0.0);
        otosYController.setDeadbandStuff(0.7, 0.0);


        apriltagRotationController = new PIDController(0.0, 0.0, 0.0);

        // ---

        // adjust these
        webInterface.addParameter("Kp_otos", 0.08, 1);
        webInterface.addParameter("Ki_otos", 0.00003, 1);
        webInterface.addParameter("Kd_otos", 0.125, 1);

        webInterface.addParameter("Kp_otosh", 0.01, 1);
        webInterface.addParameter("Ki_otosh", 0.00001, 1);
        webInterface.addParameter("Kd_otosh", 0.009, 1);

        webInterface.addParameter("Kp_aprilh", 0.0007, 1);
        webInterface.addParameter("Ki_aprilh", 0.000002, 1);
        webInterface.addParameter("Kd_aprilh", 0.004, 1);

        webInterface.addParameter("turretAngleRange", 0.49, 1);

        Thread webInterfaceThread = new Thread(webInterface);
        webInterfaceThread.start(); // start server

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterR.setDirection(DcMotor.Direction.FORWARD);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initAprilTag();

        driveController = new DriveController(imu, frontRightDrive, frontLeftDrive, backLeftDrive, backRightDrive, new PIDController(0.02, 0.001, 0.001));
        driveController.init();

        waitForStart();

        otos0Pos = myOtos.getPosition();

        // i hate this
        aToggler.currentState = true;
        // NOTE: shooter starts ON at Start; set to false for a safer default

        try {
            int loopcounter = 0;
            while (opModeIsActive()) {
                loop2(loopcounter++);
            }
        } finally {
            // release ports even if the loop throws
            webInterface.stop();

            try {
                webTelemetryStreamer.stop();
            } catch (IOException e) {}

            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    private double boolToDoubleBecauseItWontCast(boolean input) {
        return input ? 1.0 : 0.0;
    }

    public void loop2(int loopcounter) {
        telemetry.addLine("Left stick: move (field-relative)");
        telemetry.addLine("Right stick X: rotate");
        telemetry.addLine("Gamepad 2 right stick Y: turret");
        telemetry.addLine("Left bumper: intake");
        telemetry.addLine("A: feeder");
        telemetry.addLine("B: reverse");
        telemetry.addLine("Back: toggle shooter");
        telemetry.addLine("X: hold position at origin (OTOS)");
        telemetry.addLine("D-pad up: zero heading");
        telemetry.addLine("D-pad down: reset OTOS");

        // dpad down to reset otos
        if (gamepad1.dpad_down) {
            myOtos.resetTracking();
            driveController.yawZero = 0.0;
        }

        otos0Pos = myOtos.getPosition();

        shooterL_spt.update();
        shooterR_spt.update();


        driveController.setMeasuredYaw(otos0Pos.h);


        driveController.drivingPID.setCoefs(
                webInterface.getParameter("Kp_drive"),
                webInterface.getParameter("Ki_drive"),
                webInterface.getParameter("Kd_drive")
        );

        // only push shooter PIDF to the motors when one of the web parameters changed
        double kpShooter = webInterface.getParameter("Kp_shooter");
        double kiShooter = webInterface.getParameter("Ki_shooter");
        double kdShooter = webInterface.getParameter("Kd_shooter");
        double kfShooter = webInterface.getParameter("Kf_shooter");
        if (kpShooter != lastKpShooter || kiShooter != lastKiShooter
                || kdShooter != lastKdShooter || kfShooter != lastKfShooter) {
            ((DcMotorEx) shooterL).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kpShooter, kiShooter, kdShooter, kfShooter)
            );

            ((DcMotorEx) shooterR).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kpShooter, kiShooter, kdShooter, kfShooter)
            );

            lastKpShooter = kpShooter;
            lastKiShooter = kiShooter;
            lastKdShooter = kdShooter;
            lastKfShooter = kfShooter;
        }

        telemetry.addData("connection", imu.getConnectionInfo());

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("FPS: ", visionPortal.getFps());

        // turret
        double a = (double) (webInterface.getParameter("turretAngleRange") * gamepad2.right_stick_y);
        turret0.setPosition((a + 1) / 2);

        // send slower to stop network getting backed up (only every 3rd loop)
        if (loopcounter % 3 == 0) {

            webTelemetryStreamer.sendData("shooterL_current", ((DcMotorEx) shooterL).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("shooterL_speed", VELOCITY_GRAPH_SCALE * ((DcMotorEx) shooterL).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("shooterR_current", ((DcMotorEx) shooterR).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("shooterR_speed", VELOCITY_GRAPH_SCALE * ((DcMotorEx) shooterR).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("feeder_current", ((DcMotorEx) feeder).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("feeder_speed", VELOCITY_GRAPH_SCALE * ((DcMotorEx) feeder).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("intake_current", ((DcMotorEx) intake).getCurrent(CurrentUnit.MILLIAMPS) / 1000);
            webTelemetryStreamer.sendData("intake_speed", VELOCITY_GRAPH_SCALE * ((DcMotorEx) intake).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendData("SPintake_speed", webInterface.getParameter("intake_speed"));
            webTelemetryStreamer.sendData("SPshooter_speed", webInterface.getParameter("shooter_speed"));
            webTelemetryStreamer.sendData("SPfeeder_speed", webInterface.getParameter("feeder_speed"));

            // from elapsed encoder count (motorspeedtracker)
            webTelemetryStreamer.sendData("shooterL_speed_eec", shooterL_spt.rpm());
            webTelemetryStreamer.sendData("shooterR_speed_eec", shooterR_spt.rpm());

            webTelemetryStreamer.sendData("speed_backleft", VELOCITY_GRAPH_SCALE * ((DcMotorEx) backLeftDrive).getVelocity(AngleUnit.DEGREES) / 6.0);
            webTelemetryStreamer.sendData("speed_backright", VELOCITY_GRAPH_SCALE * ((DcMotorEx) backRightDrive).getVelocity(AngleUnit.DEGREES) / 6.0);
            webTelemetryStreamer.sendData("speed_frontleft", VELOCITY_GRAPH_SCALE * ((DcMotorEx) frontLeftDrive).getVelocity(AngleUnit.DEGREES) / 6.0);
            webTelemetryStreamer.sendData("speed_frontright", VELOCITY_GRAPH_SCALE * ((DcMotorEx) frontRightDrive).getVelocity(AngleUnit.DEGREES) / 6.0);

            webTelemetryStreamer.sendUpdateFrame();

            webTelemetryStreamer.sendData("otos0_x", otos0Pos.x);
            webTelemetryStreamer.sendData("otos0_y", otos0Pos.y);
            webTelemetryStreamer.sendData("otos0_h", otos0Pos.h);

            // get voltage sensor (there are two one is on the expansion hub and this is not ideal as it does not specify which one to use)
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                webTelemetryStreamer.sendData("battery_voltage", sensor.getVoltage());
                break;
            }
        }

        // reset OTOS position PIDs on the rising edge of X so they start clean
        if (gamepad1.x && !lastX) {
            otosXController.reset();
            otosYController.reset();
            otosRotationController.reset();
        }
        lastX = gamepad1.x;

        // hold mode: limit wheel acceleration and keep a minimum creep power; joystick stays instant
        driveController.maxAccelPerSec = gamepad1.x ? 6.0 : 0.0;
        driveController.minHoldPower = 0.0; // >0 makes hold mode creep forever (see sim/README.md)

        if (gamepad1.x) {
            telemetry.addLine("otosTest");

            // todo: use settable target
            otosXController.setTarget(0);
            otosYController.setTarget(0);
            otosRotationController.setTarget(0);

            double xSignal = otosXController.loop(otos0Pos.x);
            double ySignal = otosYController.loop(otos0Pos.y);
            double rotationSignal = otosRotationController.loop(otos0Pos.h);

            driveController.driveHoldFieldRelative(-ySignal, -xSignal, rotationSignal);
        } else {
            driveController.driveFieldRelative(jsrc(gamepad1.left_stick_y), -jsrc(gamepad1.left_stick_x), -jsrc(gamepad1.right_stick_x));
        }

        if (gamepad1.dpad_up) {
            driveController.yawZero = otos0Pos.h;
        }

        aToggler.update(gamepad1.back);

        double shooter_speed = webInterface.getParameter("shooter_speed");

        double reversalSignal = 1 - (2 * boolToDoubleBecauseItWontCast(gamepad1.b));

        ((DcMotorEx) intake).setVelocity(reversalSignal * boolToDoubleBecauseItWontCast(gamepad1.left_bumper) * webInterface.getParameter("intake_speed"));
        ((DcMotorEx) feeder).setVelocity(-reversalSignal * boolToDoubleBecauseItWontCast(gamepad1.a) * webInterface.getParameter("feeder_speed"));

        double shooterSpeed = boolToDoubleBecauseItWontCast(aToggler.currentState) * shooter_speed * TICKS_PER_REV / 60.0;
        ((DcMotorEx) shooterR).setVelocity(shooterSpeed);
        ((DcMotorEx) shooterL).setVelocity(shooterSpeed);

        otosXController.setCoefs(
                webInterface.getParameter("Kp_otos"),
                webInterface.getParameter("Ki_otos"),
                webInterface.getParameter("Kd_otos")
        );

        otosYController.setCoefs(
                webInterface.getParameter("Kp_otos"),
                webInterface.getParameter("Ki_otos"),
                webInterface.getParameter("Kd_otos")
        );

        otosRotationController.setCoefs(
                webInterface.getParameter("Kp_otosh"),
                webInterface.getParameter("Ki_otosh"),
                webInterface.getParameter("Kd_otosh")
        );

        apriltagRotationController.setCoefs(
                webInterface.getParameter("Kp_aprilh"),
                webInterface.getParameter("Ki_aprilh"),
                webInterface.getParameter("Kd_aprilh")
        );

        gamepad1.resetEdgeDetection();
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(
                        AprilTagGameDatabase.getBioBuzzTagLibrary()
                )
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double jsrc(double power) {
        double a = 5.1;
        double b = 0.25;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }
}
