package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Auto-Calibrated AprilTag Follower", group = "Concept")
public class AprilTagTestOpMode extends LinearOpMode {

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Self-calculated wheel trim multipliers (Default 1.0 = 100%)
    private double trimLF = 1.0, trimRF = 1.0, trimLB = 1.0, trimRB = 1.0;
    private static final List<Integer> TAG_IDs = List.of(30,31,32,33);
    private static final double DESIRED_DISTANCE = 12.0;

    private static final double SPEED_GAIN  = 0.1; //0.02
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN   = 0.01;

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Enable closed-loop PID velocity control using built-in motor encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized. Press PLAY to calibrate wheels...");
        telemetry.update();

        waitForStart();

        // 1. RUN AUTOMATIC SELF-CALIBRATION ROUTINE
        calibrateWheelSpeeds();

        // 2. APRILTAG TRACKING LOOP WITH CALIBRATED SPEEDS
        while (opModeIsActive()) {
            boolean targetFound = false;
            double drive = 0, strafe = 0, turn = 0;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null && TAG_IDs.contains(tag.id)) {
                    targetFound = true;
                    double rangeError = tag.ftcPose.range - DESIRED_DISTANCE;
                    double bearingError = tag.ftcPose.bearing;
                    double yawError = tag.ftcPose.yaw;

                    drive  = Range.clip(rangeError * SPEED_GAIN, -0.5, 0.5);
                    strafe = Range.clip(-bearingError * STRAFE_GAIN, -0.5, 0.5);
                    turn   = Range.clip(yawError * TURN_GAIN, -0.3, 0.3);
                    break;
                }
            }

            if (!targetFound) {
                drive = 0; strafe = 0; turn = 0;
            }

            moveRobotCalibrated(drive, strafe, turn);
            telemetry.update();
            sleep(10);
        }
    }

    /**
     * Self-calibration routine: Measures actual rotational velocity of each wheel
     * and scales faster wheels down to match the slowest wheel.
     */
    private void calibrateWheelSpeeds() {
        telemetry.addData("Calibration", "Measuring individual wheel velocities...");
        telemetry.update();

        int testDurationMs = 400;
        double testPower = 0.4;

        // Measure ticks per second for each motor individually
        double speedLF = measureMotorVelocity(leftFrontDrive, testPower, testDurationMs);
        double speedRF = measureMotorVelocity(rightFrontDrive, testPower, testDurationMs);
        double speedLB = measureMotorVelocity(leftBackDrive, testPower, testDurationMs);
        double speedRB = measureMotorVelocity(rightBackDrive, testPower, testDurationMs);

        // Identify the slowest wheel velocity as our benchmark
        double minSpeed = Math.min(Math.min(speedLF, speedRF), Math.min(speedLB, speedRB));

        // Calculate trim factors (Slowest Wheel / Current Wheel)
        if (minSpeed > 0) {
            trimLF = minSpeed / speedLF;
            trimRF = minSpeed / speedRF;
            trimLB = minSpeed / speedLB;
            trimRB = minSpeed / speedRB;
        }

        telemetry.addData("Self-Calibration Complete", "Trims -> LF:%.2f RF:%.2f LB:%.2f RB:%.2f",
                trimLF, trimRF, trimLB, trimRB);
        telemetry.update();
        sleep(1000);
    }

    private double measureMotorVelocity(DcMotor motor, double power, int durationMs) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int startPos = motor.getCurrentPosition();
        motor.setPower(power);
        sleep(durationMs);
        motor.setPower(0);
        int endPos = motor.getCurrentPosition();

        sleep(100); // Pause for motor to come to a complete stop
        return Math.abs((double)(endPos - startPos) / (durationMs / 1000.0));
    }

    private void moveRobotCalibrated(double drive, double strafe, double turn) {
        double lf = drive - strafe - turn;
        double rf = drive + strafe + turn;
        double lb = drive + strafe - turn;
        double rb = drive - strafe + turn;

        // Apply self-calibrated wheel trims
        lf *= trimLF;
        rf *= trimRF;
        lb *= trimLB;
        rb *= trimRB;

        // Normalize powers
        double max = Math.max(Math.abs(lf), Math.abs(rf));
        max = Math.max(max, Math.abs(lb));
        max = Math.max(max, Math.abs(rb));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
}