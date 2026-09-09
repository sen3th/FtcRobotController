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

@TeleOp(name = "Drive To AprilTag", group = "Concept")
public class AprilTagTestOpMode extends LinearOpMode {

    // Hardware variables
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;

    // Vision variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Configuration constraints
    private static final int DESIRED_TAG_ID = 583;     // Choose the tag you want to follow
    private static final double DESIRED_DISTANCE = 1.0; // How close the robot should get (inches)

    // Proportional Control Gains
    // These determine how aggressively the robot moves to correct its error
    private static final double SPEED_GAIN  =  0.02;   // Forward speed control
    private static final double STRAFE_GAIN =  0.015;  // Strafe speed control
    private static final double TURN_GAIN   =  0.01;   // Turn speed control
    private static final double MAX_AUTO_SPEED = 0.5;  // Cap maximum speed
    private static final double MAX_AUTO_STRAFE = 0.5; // Cap maximum strafe
    private static final double MAX_AUTO_TURN  = 0.3;  // Cap maximum turn

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Reverse the right side motors based on standard build practices
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            double drive  = 0;
            double strafe = 0;
            double turn   = 0;

            // Search for the target AprilTag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null && tag.id == DESIRED_TAG_ID) {
                    targetFound = true;

                    // Calculate positional errors
                    // Range error: How far are we from the desired distance?
                    double rangeError = tag.ftcPose.range - DESIRED_DISTANCE;
                    // Bearing error: How far off to the side is the tag?
                    double bearingError = tag.ftcPose.bearing;
                    // Yaw error: How skewed is the tag relative to the camera lens?
                    double yawError = tag.ftcPose.yaw;

                    // Calculate motor powers using Proportional control
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    strafe = Range.clip(-bearingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    break; // Exit loop once our specific tag is found
                }
            }

            if (!targetFound) {
                telemetry.addLine("Target not found. Halting.");
                drive = 0;
                strafe = 0;
                turn = 0;
            }

            // Apply calculated power to mecanum wheels
            moveRobot(drive, strafe, turn);
            telemetry.update();
            sleep(10);
        }
    }

    /**
     * Calculates and applies mecanum kinematics
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y - yaw;
        double rightFrontPower   =  x + y + yaw;
        double leftBackPower     =  x + y - yaw;
        double rightBackPower    =  x - y + yaw;

        // Normalize wheel powers to ensure none exceed 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the motors.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}