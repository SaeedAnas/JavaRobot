package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

public abstract class Autonomous extends LinearOpMode {
    // contains all methods to move the robot

    private static DcMotor topRight;

    private static DcMotor topLeft;

    private static DcMotor bottomLeft;

    private static DcMotor bottomRight;

    private static DcMotor armMotorLeft;

    private static DcMotor armMotorRight;

    private static Servo grabber;

    private static Servo foundationRight;

    private static Servo foundationLeft;

    private BNO055IMU imu;

    // private static CRServo armServo;

    protected void initHardware() {
        initImu();
//        leftMotor = hardwareMap.get(DcMotor.class, "left");
//        rightMotor = hardwareMap.get(DcMotor.class, "right");
//        armMotorLeft = hardwareMap.get(DcMotor.class, "armLeft");
//        armMotorRight = hardwareMap.get(DcMotor.class, "armRight");
//        foundationLeft = hardwareMap.get(Servo.class, "leftFoundation");
//        foundationRight = hardwareMap.get(Servo.class, "rightFoundation");
//        grabber = hardwareMap.get(Servo.class, "grabber");
//
        topRight = hardwareMap.get(DcMotor.class, "frontRight");
        topLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        bottomRight = hardwareMap.get(DcMotor.class, "rearRight");
        // armServo = hardwareMap.get(CRServo.class, "armServo");
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reverse right motors
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightMotor is upside-down
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // reset the encoder
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run using encoder
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Ready!", "Press Start.");
        telemetry.update();
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

        telemetry.addData("Status: ", "Calibrating");
        telemetry.update();
        imu.initialize(parameters);

        telemetry.addData("Status: ", "Imu Calibration Ready");
        telemetry.update();

    }


    protected void brake() {
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


    private void turnLeftByGyro(double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
        topLeft.setPower(-motorPower);
        bottomLeft.setPower(-motorPower);
        topRight.setPower(motorPower);
        bottomRight.setPower(motorPower);
        while (opModeIsActive() && (angleTurned < targetDegree)) {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

    private void turnRightByGyro(double motorPower, double targetDegree) {
        double zeroReference = getGyroYAngle();
        double angleTurned = 0;
        topLeft.setPower(motorPower);
        bottomLeft.setPower(motorPower);
        topRight.setPower(-motorPower);
        bottomRight.setPower(-motorPower);
        while (opModeIsActive() && (angleTurned > (-targetDegree))) {
            double currentAngle = getGyroYAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("AngleTurned", angleTurned);
            telemetry.addData("Target", targetDegree);
            telemetry.update();
        }
        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        telemetry.addData("Power = 0", 0);
        telemetry.update();
    }

    protected void move(Direction direction, double distance, double power) {
        if (opModeIsActive()) {
            DcMotor[] motors = direction.getMotors();
            double[] targets = direction.getTarget(distance, motors);
            direction.setPower(power);
            while (opModeIsActive() && direction.hasNotReached(targets, motors)) {
                    direction.setPower(power);
                    telemetry.addData("Direction: ", direction);
                    telemetry.update();
            }
            Direction.stopRobot(motors);
        }
    }

    protected void autoCorrectMove(Direction direction, double distance, double power) {
        if (opModeIsActive()) {
            double degree = getGyroYAngle();
            double currentDegree;
            DcMotor[] motors = direction.getMotors();
            double[] targets = direction.getTarget(distance, motors);
            direction.setPower(power);
            while (opModeIsActive() && direction.hasNotReached(targets, motors)) {
                currentDegree = getGyroYAngle();
                if (currentDegree < degree - DEGREE_THRESHOLD) {
                    correctPosition(degree, power);
                }
                else if (currentDegree > degree + DEGREE_THRESHOLD) {
                    correctPosition(degree, power);
                }
                else {
                    direction.setPower(power);
                    telemetry.addData("Direction: ", direction);
                    telemetry.update();
                }
            }
            Direction.stopRobot(motors);
        }
    }

    private void correctPosition(double degree, double power) {
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (isRight(degree)) {
            topLeft.setPower(power);
            bottomLeft.setPower(power);
            topRight.setPower(-power);
            bottomRight.setPower(-power);
        } else {
            topLeft.setPower(-power);
            bottomLeft.setPower(-power);
            topRight.setPower(power);
            bottomRight.setPower(power);
        }
            double currentAngle = getGyroYAngle();
            while(opModeIsActive() && ((currentAngle < degree - DEGREE_THRESHOLD) && (currentAngle > degree + DEGREE_THRESHOLD))) {
                telemetry.addData("Status", "Correcting");
                telemetry.update();
                currentAngle = getGyroYAngle();
            }

        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public enum Direction {

        FORWARD {
            @Override
            public void setPower(double power) {
                topRight.setPower(power);
                topLeft.setPower(power);
                bottomRight.setPower(power);
                bottomLeft.setPower(power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                double[] targets = new double[motors.length];
                for (int i = 0; i < targets.length; i++) {
                    targets[i] = motors[i].getCurrentPosition() + (distance * TICKS_PER_INCH_STRAIGHT);
                }
                return targets;
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                boolean hasNotReached = true;
                for (int i = 0; i < targets.length; i++) {
                    if (motors[i].getCurrentPosition() > targets[i]) {
                        hasNotReached = false;
                        break;
                    }
                }
                return hasNotReached;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        BACKWARD {
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                double[] targets = new double[motors.length];
                for (int i = 0; i < targets.length; i++) {
                    targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
                }
                return targets;
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                boolean hasNotReached = true;
                for (int i = 0; i < targets.length; i++) {
                    if (motors[i].getCurrentPosition() < targets[i]) {
                        hasNotReached = false;
                        break;
                    }
                }
                return hasNotReached;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        LEFT {
            @Override
            public void setPower(double power) {
                topRight.setPower(power);
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
                bottomLeft.setPower(power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                double[] targets = new double[motors.length];
                for (int i = 0; i < targets.length; i++) {
                    targets[i] = motors[i].getCurrentPosition() - (distance * TICKS_PER_INCH_STRAIGHT);
                }
                return targets;
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        RIGHT {
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                topLeft.setPower(power);
                bottomRight.setPower(power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                return new double[0];
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, topLeft, bottomLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        FORWARD_LEFT {
            @Override
            public void setPower(double power) {
                topRight.setPower(power);
                bottomLeft.setPower(power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                return new double[0];
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, bottomLeft};
            }
        },
        // TODO Change the getTarget and hasNotReached
        FORWARD_RIGHT {
            @Override
            public void setPower(double power) {
                topLeft.setPower(power);
                bottomRight.setPower(power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                return new double[0];
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topLeft, bottomRight};
            }
        },
        // TODO Change the getTarget and hasNotReached
        BACKWARD_LEFT {
            @Override
            public void setPower(double power) {
                topRight.setPower(-power);
                bottomLeft.setPower(-power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                return new double[0];
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topRight, bottomLeft};
            }
        },
        // TODO Change the getTarget and hasNotReached
        BACKWARD_RIGHT {
            @Override
            public void setPower(double power) {
                topLeft.setPower(-power);
                bottomRight.setPower(-power);
            }

            @Override
            public double[] getTarget(double distance, DcMotor[] motors) {
                return new double[0];
            }

            @Override
            public boolean hasNotReached(double[] targets, DcMotor[] motors) {
                return false;
            }

            @Override
            public DcMotor[] getMotors() {
                return new DcMotor[]{topLeft, bottomRight};
            }
        };


        public abstract void setPower(double power);

        public abstract double[] getTarget(double distance, DcMotor[] motors);

        public abstract boolean hasNotReached(double[] targets, DcMotor[] motors);

        public abstract DcMotor[] getMotors();

        public static void stopRobot(DcMotor[] motors) {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
        }

    }

    protected void calibrate() {
        topLeft.setPower(0.5);
        sleep(1000);
        topLeft.setPower(-0.5);
        sleep(1000);
        topRight.setPower(0.5);
        sleep(1000);
        topRight.setPower(-0.5);
        sleep(1000);
        bottomLeft.setPower(0.5);
        sleep(1000);
        bottomLeft.setPower(-0.5);
        sleep(1000);
        bottomRight.setPower(0.5);
        sleep(1000);
        bottomRight.setPower(-0.5);
        sleep(1000);
        rotateMotorsOnce(FORWARD, 1);
        sleep(1000);
        rotateMotorsOnce(BACKWARD, 1);
        sleep(1000);
        rotateMotorsOnce(LEFT, 1);
        sleep(1000);
        rotateMotorsOnce(RIGHT, 1);
        sleep(1000);
        rotateMotorsOnce(FORWARD_LEFT, 1);
        sleep(1000);
        rotateMotorsOnce(BACKWARD_LEFT, 1);
        sleep(1000);
        rotateMotorsOnce(FORWARD_RIGHT, 1);
        sleep(1000);
        rotateMotorsOnce(BACKWARD_RIGHT, 1);
        sleep(1000);

    }

    protected void rotateMotorsOnce(Direction direction, double power) {
        DcMotor[] motors = direction.getMotors();
        direction.setPower(power);
        sleep(1000);
        Direction.stopRobot(motors);
    }
}


