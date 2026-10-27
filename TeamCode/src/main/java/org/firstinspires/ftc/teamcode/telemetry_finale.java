package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="auto wheels with fallback")
public class telemetry_finale extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // --- ENCODER CONVERSION CONSTANTS ---
    // REV UltraPlanetary HD Hex motor output ticks per revolution (e.g., 537.7 ticks for 19.2:1 cartridge)
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm wheel diameter
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // --- VISION TARGETING CONSTANTS ---
    private static final double DESIRED_DISTANCE = 12.0; // Distance to stop in inches
    private static final double SPEED_GAIN = 0.04;
    private static final double TURN_GAIN = 0.02;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_TURN = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) {
            libraryBuilder.addTag(id, "Tag_" + id, 3.5, DistanceUnit.INCH);
        }
        AprilTagLibrary customLibrary = libraryBuilder.build();

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.50)
                .setTagLibrary(customLibrary)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCamera(hardwareMap.get(WebcamName.class, "Cam1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // Start 3-second timer to scan for AprilTags
        ElapsedTime timer = new ElapsedTime();
        boolean tagFound = false;

        while (opModeIsActive() && timer.seconds() < 3.0 && !tagFound) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection tag : detections) {
                if (tag.id >= 30 && tag.id <= 33 && tag.ftcPose != null) {
                    tagFound = true;
                    break;
                }
            }
            if (!tagFound) {
                telemetry.addData("Status", "Scanning for AprilTags... %.1fs remaining", 3.0 - timer.seconds());
                telemetry.update();
            }
        }

        // Branching logic: Vision vs. Encoder Fallback
        if (tagFound) {
            telemetry.addLine("AprilTag detected! Driving using camera alignment...");
            telemetry.update();
            driveWithVision(aprilTag);
        } else {
            telemetry.addLine("No tag detected within 3s! Executing encoder fallback move...");
            telemetry.update();
            driveDistanceEncoder(24.0, 0.4); // Fallback: drive 24 inches forward at 0.4 power
        }
    }

    private void driveWithVision(AprilTagProcessor aprilTag) {
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : detections) {
                if (tag.id >= 30 && tag.id <= 33 && tag.ftcPose != null) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag != null) {
                double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = targetTag.ftcPose.bearing;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                frontLeft.setPower(drive + turn);
                backLeft.setPower(drive + turn);
                frontRight.setPower(drive - turn);
                backRight.setPower(drive - turn);

                // Target reached threshold
                if (Math.abs(rangeError) < 1.0 && Math.abs(headingError) < 2.0) {
                    stopMotors();
                    telemetry.addLine("Target position reached!");
                    telemetry.update();
                    break;
                }
            } else {
                stopMotors();
            }
        }
    }

    private void driveDistanceEncoder(double distanceInches, double power) {
        resetEncoders();

        int targetTicks = (int) (distanceInches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(targetTicks);
        frontRight.setTargetPosition(targetTicks);
        backLeft.setTargetPosition(targetTicks);
        backRight.setTargetPosition(targetTicks);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()) {
            telemetry.addData("Fallback Mode", "Driving %s inches", distanceInches);
            telemetry.addData("Current Ticks", frontLeft.getCurrentPosition());
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.update();
        }

        stopMotors();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}