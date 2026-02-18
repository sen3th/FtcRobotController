package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name ="MotorController", group = "TeleOp")
public class MotorController extends OpMode {

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;

    @Override
    public void init() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");


        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){
        double leftJoystickValue = -gamepad1.left_stick_y;
        double rightJoystickValue = -gamepad1.right_stick_y;

        backLeftMotor.setPower(leftJoystickValue);
        frontLeftMotor.setPower(leftJoystickValue);

        backRightMotor.setPower(rightJoystickValue);
        frontRightMotor.setPower(rightJoystickValue);

        telemetry.addData("Left", leftJoystickValue);
        telemetry.addData("Right", rightJoystickValue);
        telemetry.update();
    }
}

