package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Dual Servo Mechanism Control", group = "TeleOp")
public class Shooter2 extends LinearOpMode {

    private DcMotor mechanismMotor = null;

    @Override
    public void runOpMode() {
        mechanismMotor = hardwareMap.get(DcMotor.class, "mechanism_motor");

        mechanismMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Ready for start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. MOTOR CONTROL (Left Stick Y)
            double motorPower = -gamepad1.left_stick_y;
            mechanismMotor.setPower(motorPower);

            // Telemetry output
            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.update();

            sleep(20);
        }
    }
}