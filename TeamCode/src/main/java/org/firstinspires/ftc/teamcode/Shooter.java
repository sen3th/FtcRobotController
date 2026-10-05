
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="testshooter")
public class Shooter extends LinearOpMode {
    double speedmultiplier = 0.65;
    double limit = 2.5;

    boolean mode = false;
    private DcMotor leftShooter;
    private Servo leftServo;
    private Servo rightServo;

    @Override
    public void runOpMode() {
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftShooter.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addLine("- Shooter Ready - ");
        telemetry.update();

        waitForStart();

        boolean debounce = false;

        while (opModeIsActive()) {
            double shooter = gamepad1.left_trigger;
            boolean change_gear = gamepad1.right_trigger_pressed;

            boolean increaser = gamepad1.y;
            boolean decreaser = gamepad1.x;

            if (change_gear) {
                mode = !mode;

                if (mode) {
                    telemetry.addData("put it in forward terry", "");
                    telemetry.update();
                    leftShooter.setDirection(DcMotor.Direction.FORWARD);
                }
                else{
                    telemetry.addData("put it in reverse terry", "");
                    telemetry.update();
                    leftShooter.setDirection(DcMotor.Direction.REVERSE);
                }

                while (change_gear) {
                    change_gear = gamepad1.right_trigger_pressed;

                    if (gamepad1.a || gamepad1.b || decreaser || increaser) {
                        break;
                    }
                }


                telemetry.addData("Cooldown", "- yes");
                telemetry.update();
                sleep(200);
                telemetry.addData("Cooldown", "- no");

            }

            if (increaser && speedmultiplier <= limit) {
                speedmultiplier += 0.0001;
            }

            if (decreaser && speedmultiplier >= 0.15) {
                speedmultiplier -= 0.0001;
            }

            leftShooter.setPower(shooter*speedmultiplier);

            ServoDetection(gamepad1.a, gamepad1.b);

            telemetry.addData("Speed", speedmultiplier);
            telemetry.addData("Left Servo Pos", leftServo.getPosition());
            telemetry.addData("Right Servo Pos", rightServo.getPosition());
            telemetry.update();
        }
    };

    private void ServoDetection(boolean bigger, boolean smaller) {
        if (bigger) {
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        }

        if (smaller) {
            leftServo.setPosition(0.0);
            rightServo.setPosition(0.0);
        }
    }
}
