package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="gamepad", group="Forward")
public class Gamepad extends LinearOpMode {

    DcMotor leftMotor = hardwareMap.dcMotor.get("m1");
    DcMotor rightMotor = hardwareMap.dcMotor.get("m2");
    Servo servoTest = hardwareMap.servo.get("servoTest");

@Override
    public void runOpMode() throws InterruptedException {

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Encoder Value: ", (rightMotor.getCurrentPosition()
                    +leftMotor.getCurrentPosition())/2);
            // check to see if we need to move the servo.
            if(gamepad1.y) {
                // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            //drive fwd/backward
            double power = -gamepad1.left_stick_y;
            setDrivePower(power, power);

            //turns
            double turn = gamepad1.right_stick_x;
            setDrivePower(turn, -turn);

        }

    }

    public void setDrivePower(double leftPower, double rightPower) {
        rightMotor.setPower(rightPower);
        leftMotor.setPower(leftPower);
    }

}
