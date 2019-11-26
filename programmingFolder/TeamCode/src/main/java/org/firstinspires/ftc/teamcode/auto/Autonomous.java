package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    private BNO055IMU imu;

    // private static CRServo armServo;

    protected void initHardware() {
        initImu();
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
        telemetry.addData("Ready!", "Press Start.");
        // wait for play
        waitForStart();
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Mode", "calibrating...");
            telemetry.update();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }


    protected void brake() {
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Foundation code
    protected void grabFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_DOWN);
        foundationRight.setPosition(RIGHT_FOUNDATION_DOWN);
    }

    protected void releaseFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_UP);
        foundationRight.setPosition(RIGHT_FOUNDATION_UP);
    }

    protected void grab() {
        grabber.setPosition(GRABBER_GRAB);
    }

    protected void release() {
        grabber.setPosition(GRABBER_RELEASE);
    }

    // arm code
    protected void whileArm(double mm, double power) {
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

    // TODO Test out this code

    protected void autoCorrectingDrive(double power, double distance) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            double degree = getGyroYAngle();
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // for forward
            if (distance > 0) {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                while (opModeIsActive() && leftMotor.getCurrentPosition() < newLeftTarget && rightMotor.getCurrentPosition() < newRightTarget) {
                    double currentDegree = getGyroYAngle();
                    if (currentDegree < degree - DEGREE_THRESHOLD) {
                        backToPosition(degree, power);
                    } else if (currentDegree > degree + DEGREE_THRESHOLD) {
                        backToPosition(degree, power);
                    } else {
                        leftMotor.setPower(power);
                        rightMotor.setPower(power);
                    }
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            } else {
                leftMotor.setPower(-power);
                rightMotor.setPower(-power);
                while (opModeIsActive() && leftMotor.getCurrentPosition() > newLeftTarget && rightMotor.getCurrentPosition() > newRightTarget) {
                    double currentDegree = getGyroYAngle();
                    if (currentDegree < degree - DEGREE_THRESHOLD) {
                        backToPosition(degree, power);
                    } else if (currentDegree > degree + DEGREE_THRESHOLD) {
                        backToPosition(degree, power);
                    } else {
                        leftMotor.setPower(-power);
                        rightMotor.setPower(-power);
                    }
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }
    }

    private void backToPosition(double degree, double power) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if the fastest way is to go right
        if(isRight(degree)) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            double currentAngle = getGyroYAngle();
            while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) && (currentAngle > degree + DEGREE_THRESHOLD))) {
                currentAngle = getGyroYAngle();
                telemetry.addData("Status", "Correcting...");
                telemetry.update();
            }
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // the fastest way to go is left
        else {
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            double currentAngle = getGyroYAngle();
            while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) && (currentAngle > degree + DEGREE_THRESHOLD))) {
                currentAngle = getGyroYAngle();
                telemetry.addData("Status", "Correcting...");
                telemetry.update();
            }
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private boolean isRight(double degree) {
        double currentAngle = getGyroYAngle();
        boolean isRight = false;
        // left increases
        // right decreases
        // -179 -> 179
        // if the current angle is negative and the degree > 0, then the fastest way to get to the degree is to go the opposite way.
        if (currentAngle < 0 && degree > 0) {
             isRight = true;
        }
        // we dont need these two because isRight is already false -> saves some time
//        else if (currentAngle > 0 && degree < 0) {
//             isRight = false;
//        } else if (currentAngle < degree) {
//            isRight = false;
//        }
        else if (currentAngle > degree) {
            isRight = true;
        }
        return isRight;
    }


    protected void drive(double power, double distance) {
        drive(power, distance, distance);

    }

    // Drive Using Encoder
    private void drive(double power, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            if (leftInches < 0) {

                leftMotor.setPower(-power);
                rightMotor.setPower(-power);
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

// 179 -> -179
// y value
// right decreasing
// left increasing

    private double getGyroYAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return (angles.secondAngle);
    }

    private double adjustedAngle(double zeroReference, double currentAngle) {
        double adjusted = currentAngle - zeroReference;
        if (adjusted < -179) {
            adjusted += 360;
        } else if (adjusted > 180) {
            adjusted -= 360;
        }
        return adjusted;
    }

    protected void turnByGyro(double motorPower, double targetDegree) {
        if (targetDegree < 0) {
            turnLeftByGyro(motorPower, -(targetDegree - CORRECTION));
        } else {
            turnRightByGyro(motorPower, -(targetDegree - CORRECTION));
        }
    }



    private void turnLeftByGyro (double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
       rightMotor.setPower(-motorPower);
       leftMotor.setPower(motorPower);
        while( opModeIsActive() && (angleTurned < targetDegree))
        {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        rightMotor.setPower(0);
         leftMotor.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

    private void turnRightByGyro (double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
        rightMotor.setPower(motorPower);
        leftMotor.setPower(-motorPower);
        while( opModeIsActive() && (angleTurned > (-targetDegree)))
        {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

//    private void resetEncoder() {
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void encoderTurn(double power, double degree, double timeoutS) {
//        if (degree < 0) {
//            encoderTurnLeft(power, -degree, timeoutS);
//        } else {
//            encoderTurnRight(power, degree, timeoutS);
//        }
//    }
//
//    public void encoderTurnOneWheel(double power, double degree, double timeoutS) {
//        if (degree < 0) {
//            encoderTurnLeftOne(power, degree, timeoutS);
//        } else {
//            encoderTurnRightOne(power, degree, timeoutS);
//        }
//    }
//
//    public void encoderTurnLeft(double power, double degree, double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        resetEncoder();
//
//        if (opModeIsActive()) {
//            newLeftTarget = -leftMotor.getCurrentPosition() + (int) calcTurn(degree);
//            telemetry.addData("leftTarget", newLeftTarget);
//            newRightTarget = rightMotor.getCurrentPosition() - (int) calcTurn(degree);
//            telemetry.addData("rightTarget", newRightTarget);
//            leftMotor.setTargetPosition(newLeftTarget);
//            rightMotor.setTargetPosition(newRightTarget);
//
//
//            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            leftMotor.setPower(Math.abs(power));
//            rightMotor.setPower(Math.abs(power));
//
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
//                    (leftMotor.isBusy() && rightMotor.isBusy())) {
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
//                telemetry.update();
//
//            }
//
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            resetEncoder();
//        }
//
//
//    }
//
//    public void encoderTurnRight(double power, double degree, double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        resetEncoder();
//
//
//        if (opModeIsActive()) {
//            newLeftTarget = leftMotor.getCurrentPosition() - (int) calcTurn(degree);
//            telemetry.addData("leftTarget", newLeftTarget);
//            newRightTarget = -rightMotor.getCurrentPosition() + (int) calcTurn(degree);
//            telemetry.addData("rightTarget", newRightTarget);
//            leftMotor.setTargetPosition(newLeftTarget);
//            rightMotor.setTargetPosition(newRightTarget);
//
//
//            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            leftMotor.setPower(Math.abs(power));
//            rightMotor.setPower(Math.abs(power));
//
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
//                    (leftMotor.isBusy() && rightMotor.isBusy())) {
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
//                telemetry.update();
//
//            }
//
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            resetEncoder();
//        }
//
//
//    }
//
//    private double calcTurn(double degree) {
//        return (Math.PI * ROBOT_WIDTH * (degree / 360)) * COUNTS_PER_INCH;
//    }
//
//    public void encoderTurnLeftOne(double power, double degree, double timeoutS) {
//        int newRightTarget;
//
//        resetEncoder();
//
//        if (opModeIsActive()) {
//            newRightTarget = rightMotor.getCurrentPosition() - (int) calcTurnRadius(degree);
//            telemetry.addData("rightTarget", newRightTarget);
//            rightMotor.setTargetPosition(newRightTarget);
//
//            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            rightMotor.setPower(Math.abs(power));
//
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
//                    (rightMotor.isBusy())) {
//                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
//                telemetry.update();
//
//            }
//            rightMotor.setPower(0);
//            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            resetEncoder();
//        }
//
//
//    }
//
//
//    public void encoderTurnRightOne(double power, double degree, double timeoutS) {
//        int newRightTarget;
//
//        resetEncoder();
//
//        if (opModeIsActive()) {
//            newRightTarget = leftMotor.getCurrentPosition() - (int) calcTurnRadius(degree);
//            telemetry.addData("rightTarget", newRightTarget);
//            leftMotor.setTargetPosition(newRightTarget);
//
//            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            leftMotor.setPower(Math.abs(power));
//
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
//                    (rightMotor.isBusy())) {
//                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
//                telemetry.update();
//
//            }
//            leftMotor.setPower(0);
//            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            resetEncoder();
//        }
//    }
//
//
//    private double calcTurnRadius(double degree) {
//        return (Math.PI * ROBOT_WIDTH * 2 * (degree / 360)) * COUNTS_PER_INCH;
//    }


//


}
