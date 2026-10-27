package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "New_Webcam", group = "Concept")
public class New_Webcam extends LinearOpMode {

    //private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
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
        //leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        //rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        //leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        //rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Enable closed-loop PID velocity control using built-in motor encoders
        // setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized. Press PLAY to calibrate wheels...");
        telemetry.update();

        waitForStart();

        telemetry.addData("status", "started");
        telemetry.update();

        // 1. RUN AUTOMATIC SELF-CALIBRATION ROUTINE
        // calibrateWheelSpeeds();

        // 2. APRILTAG TRACKING LOOP WITH CALIBRATED SPEEDS
        while (opModeIsActive()) {
            boolean targetFound = false;

            double drive = 0, strafe = 0, turn = 0;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection tag : currentDetections) {
                telemetry.addData("caption", tag.id);
                if (tag.metadata != null && TAG_IDs.contains(tag.id)) {
                    targetFound = true;
                    double rangeError = tag.ftcPose.range - DESIRED_DISTANCE;
                    double bearingError = tag.ftcPose.bearing;
                    double yawError = tag.ftcPose.yaw;

                    telemetry.addData("Range", tag.ftcPose.range);

                    drive  = Range.clip(rangeError * SPEED_GAIN, -0.5, 0.5);
                    strafe = Range.clip(-bearingError * STRAFE_GAIN, -0.5, 0.5);
                    turn   = Range.clip(yawError * TURN_GAIN, -0.3, 0.3);
                    break;
                }
            }

            if (!targetFound) {
                drive = 0; strafe = 0; turn = 0;
            }

            // moveRobotCalibrated(drive, strafe, turn);

            if (targetFound) {
                telemetry.addData("Status", "Found Target");
            }

            telemetry.update();
            sleep(10);
        }
    }
}