package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Figure Eight Small Area", group = "Autonomous")
public class the_Eight extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Both motors run in default FORWARD direction (no direction reversal)
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready for tight-space Figure 8.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            setDrivePower(0.2, 1, 2500);

            setDrivePower(0.5, 0.4, 6500);

            setDrivePower(0.45, 1, 500);

            setDrivePower(0.008, 1, 1800);

            setDrivePower(0.004, 1, 2400);

            setDrivePower(0.45, 1, 1000);

            stopRobot();
        }
    }

    private void setDrivePower(double leftPower, double rightPower, long durationMs) {
        if (!opModeIsActive()) return;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        sleep(durationMs);
    }

    private void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}