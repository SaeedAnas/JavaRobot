package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.auto.Constants.*;

public abstract class Autonomous extends LinearOpMode {
    // contains all methods to move the robot

    private static DcMotor leftMotor;

    private static DcMotor rightMotor;

    private static DcMotor armMotorLeft;

    private static DcMotor armMotorRight;

    private static Servo grabber;

    private static Servo foundationRight;

    private static Servo foundationLeft;

    // private static CRServo armServo;

    private ElapsedTime runtime = new ElapsedTime();

    public void initHardware() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        armMotorLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armMotorRight = hardwareMap.get(DcMotor.class, "armRight");
        foundationLeft = hardwareMap.get(Servo.class, "leftFoundation");
        foundationRight = hardwareMap.get(Servo.class, "rightFoundation");
        grabber = hardwareMap.get(Servo.class, "grabber");
        // armServo = hardwareMap.get(CRServo.class, "armServo");

        // rightMotor is upside-down
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // reset the encoder
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run using encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait for play
        waitForStart();
    }

    public void brake() {
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Foundation code
    public void grabFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_DOWN);
        foundationRight.setPosition(RIGHT_FOUNDATION_DOWN);
    }

    public void releaseFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_UP);
        foundationRight.setPosition(RIGHT_FOUNDATION_UP);
    }

    public void grab() {
        grabber.setPosition(GRABBER_GRAB);
    }

    public void release() {
        grabber.setPosition(GRABBER_RELEASE);
    }

    // arm code
    public void whileArm(double mm, double power) {
        double target = ((armMotorLeft.getCurrentPosition() + (mm * TICKS_PER_MM_ARM)) + (armMotorRight.getCurrentPosition() + (mm * TICKS_PER_MM_ARM))) / 2;
        if (mm < 0) {
            armMotorLeft.setPower(-power);
            armMotorRight.setPower(-power);
            while (opModeIsActive() && (armMotorLeft.getCurrentPosition() > target && armMotorRight.getCurrentPosition() > target)) {
                telemetry.addData("MovingArm", armMotorLeft.getCurrentPosition());
                telemetry.addData("TARGET", target);
                telemetry.update();
            }
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
        } else {
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
            while (opModeIsActive() && (armMotorLeft.getCurrentPosition() < target && armMotorRight.getCurrentPosition() < target)) {
                telemetry.addData("MovingArm", armMotorLeft.getCurrentPosition());
                telemetry.addData("TARGET", target);
                telemetry.update();
            }
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
        }
    }

    public void drive(double power, double distance) {
        drive(power, distance, distance);

    }

    // Drive Using Encoder
    public void drive(double power, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            if (leftInches < 0) {

                leftMotor.setPower(power);
                rightMotor.setPower(power);
                while (opModeIsActive() && leftMotor.getCurrentPosition() > newLeftTarget && rightMotor.getCurrentPosition() > newRightTarget) {
                    telemetry.addData("CurrentLeft: ", leftMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newLeftTarget);
                    telemetry.addData("CurrentRight: ", rightMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newRightTarget);
                    telemetry.update();
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            } else {
                leftMotor.setPower(power);
                rightMotor.setPower(power);

                while (opModeIsActive() && leftMotor.getCurrentPosition() < newLeftTarget && rightMotor.getCurrentPosition() < newRightTarget) {
                    telemetry.addData("CurrentLeft: ", leftMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newLeftTarget);
                    telemetry.addData("CurrentRight: ", rightMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newRightTarget);
                    telemetry.update();
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }


        }


    }

    private void resetEncoder() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn(double power, double degree, double timeoutS) {
        if (degree < 0) {
            encoderTurnLeft(power, -degree, timeoutS);
        } else {
            encoderTurnRight(power, degree, timeoutS);
        }
    }

    public void encoderTurnOneWheel(double power, double degree, double timeoutS) {
        if (degree < 0) {
            encoderTurnLeftOne(power, degree, timeoutS);
        } else {
            encoderTurnRightOne(power, degree, timeoutS);
        }
    }

    public void encoderTurnLeft(double power, double degree, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        resetEncoder();

        if (opModeIsActive()) {
            newLeftTarget = -leftMotor.getCurrentPosition() + (int) calcTurn(degree);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = rightMotor.getCurrentPosition() - (int) calcTurn(degree);
            telemetry.addData("rightTarget", newRightTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);


            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftMotor.setPower(Math.abs(power));
            rightMotor.setPower(Math.abs(power));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoder();
        }


    }

    public void encoderTurnRight(double power, double degree, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        resetEncoder();


        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() - (int) calcTurn(degree);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = -rightMotor.getCurrentPosition() + (int) calcTurn(degree);
            telemetry.addData("rightTarget", newRightTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);


            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftMotor.setPower(Math.abs(power));
            rightMotor.setPower(Math.abs(power));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoder();
        }


    }

    private double calcTurn(double degree) {
        return (Math.PI * ROBOT_WIDTH * (degree / 360)) * COUNTS_PER_INCH;
    }

    public void encoderTurnLeftOne(double power, double degree, double timeoutS) {
        int newRightTarget;

        resetEncoder();

        if (opModeIsActive()) {
            newRightTarget = rightMotor.getCurrentPosition() - (int) calcTurnRadius(degree);
            telemetry.addData("rightTarget", newRightTarget);
            rightMotor.setTargetPosition(newRightTarget);

            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            rightMotor.setPower(Math.abs(power));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (rightMotor.isBusy())) {
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();

            }
            rightMotor.setPower(0);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoder();
        }


    }


    public void encoderTurnRightOne(double power, double degree, double timeoutS) {
        int newRightTarget;

        resetEncoder();

        if (opModeIsActive()) {
            newRightTarget = leftMotor.getCurrentPosition() - (int) calcTurnRadius(degree);
            telemetry.addData("rightTarget", newRightTarget);
            leftMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftMotor.setPower(Math.abs(power));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (rightMotor.isBusy())) {
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();

            }
            leftMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoder();
        }
    }


    private double calcTurnRadius(double degree) {
        return (Math.PI * ROBOT_WIDTH * 2 * (degree / 360)) * COUNTS_PER_INCH;
    }


}
