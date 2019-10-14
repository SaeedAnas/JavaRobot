package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Autonomous extends LinearOpMode {
    DcMotor rightMotor = hardwareMap.dcMotor.get("m1");
    DcMotor leftMotor = hardwareMap.dcMotor.get("m2");

    double degrees = rightMotor.getCurrentPosition() * 1440 / 360;

    @Override

    public void runOpMode() throws InterruptedException {
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            forward(45, degrees, rightMotor, leftMotor);

        }


    }

    public static void forward(double x, double degrees, DcMotor rightMotor, DcMotor leftMotor) {

        while (degrees < x) {
            rightMotor.setPower(1);
            leftMotor.setPower(1);


        }

    }
}


