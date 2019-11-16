
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

// Docs
// https://ftctechnh.github.io/ftc_app/doc/javadoc/org/firstinspires/ftc/robotcore/external/navigation/package-summary.html

@Autonomous
public class Auto extends LinearOpMode {

    // WARNING : CODE HAS NOT BEEN TESTED AND MAY NOT WORK

    // TODO : Measure the Width of the drive train (inches) and input the value here

    private static final double ROBOT_WIDTH = 18;

    // runtime
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();

    // Encoder variables
    private static final double
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 0.5, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 4.0,
            COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            DRIVE_SPEED = 0.4,
            TURN_SPEED = 0.5,
            ARM_SPEED = 0.3,
            LEFT_FOUNDATION_DOWN = 0.4,
            LEFT_FOUNDATION_UP = 0.9,
            RIGHT_FOUNDATION_DOWN = 0.6,
            RIGHT_FOUNDATION_UP = 0.1,
            GRABBER_GRAB = 0.7,
            GRABBER_RELEASE = 0.0,
            GEAR_IN = 32,
            GEAR_OUT = 16,
            GEAR_RATIO_ARM = GEAR_OUT/GEAR_IN,
            P_TICKS = 28,
            P_DISTANCE_PER_ROTATION = 20.8,
            TICKS_PER_MM_ARM = (GEAR_RATIO_ARM * COUNTS_PER_MOTOR_REV)/P_DISTANCE_PER_ROTATION,
            TICKS_PER_INCH_ARM = TICKS_PER_MM_ARM / 25.4,
            BLOCK_HEIGHT_MM = 101.6,
            BLOCK_HEIGHT_INCHES = BLOCK_HEIGHT_MM / 25.4;



    // Hardware
    private DcMotor leftMotor;

    private DcMotor rightMotor;

    private DcMotor armMotor;

    private Servo grabber;

    private Servo foundationRight;

    private Servo foundationLeft;

    private CRServo armServo;


    // IMU sensor variables
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init the hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        armMotor = hardwareMap.get(DcMotor.class,"arm");
        foundationLeft = hardwareMap.get(Servo.class, "leftFoundation");
        foundationRight = hardwareMap.get(Servo.class, "rightFoundation");
        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo = hardwareMap.get(CRServo.class, "armServo");

        // rightMotor is upside-down
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // reset the encoder
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motor to run using encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the imu
        //initImu();

        final double DISTANCE_TO_FOUNDATION_START = 12;
        final double FOUNDATION_SERVO_DEGREE = 0.5;
        final double DISTANCE_TO_FIRST_BLOCK = 100;
        final double DISTANCE_TO_FOUNDATION_BLOCK = 200;

        // wait for play
        waitForStart();

        // run
        int count = 1;
        while (opModeIsActive()) {
             printStatus();
//             releaseFoundation();
//             whileDrive(-DRIVE_SPEED,-30,-30,5.0);
//             grabFoundation();
//             whileDrive(1, 30,30,5.0);
//             releaseFoundation();
//             encoderTurnLeft(TURN_SPEED, 90,5.0);
//            whileDrive(DRIVE_SPEED, 10,11,5.0);
//            encoderTurnLeft(TURN_SPEED, 90,5.0);
//            whileDrive(DRIVE_SPEED, 15,15,5.0);
//            encoderTurnLeft(TURN_SPEED, 90,5.0);
//            whileDrive(DRIVE_SPEED, 15,15,5.0);
//            encoderTurnLeft(TURN_SPEED, 90,5.0);
//            whileDrive(DRIVE_SPEED, 30, 30,5.0);
            if(count == 1) {
                encoderTurnLeft(TURN_SPEED, 90, 5.0);
                encoderTurnRight(TURN_SPEED, 90, 5.0);
                whileDrive(DRIVE_SPEED, 30, 30, 5.0);
                moveArmMotor(10,0.2);
                moveArmMotor(-10,0.2);
            }
            grabber.setPosition(GRABBER_GRAB);
            sleep(1000);
            grabber.setPosition(GRABBER_RELEASE);
            sleep(1000);
            count++;
            moveArmMotor(10, 0.2);
            }

        }
        // stop
    }



    private void moveArmServo(double power) {
        armServo.setPower(power);
    }


    // Foundation code
    private void grabFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_DOWN);
        foundationRight.setPosition(RIGHT_FOUNDATION_DOWN);
    }
    private void releaseFoundation() {
        foundationLeft.setPosition(LEFT_FOUNDATION_UP);
        foundationRight.setPosition(RIGHT_FOUNDATION_UP);
    }

    // arm code

    private void moveArmMotor(double mm, double power) {
        int target = armMotor.getCurrentPosition() + (int)(mm * TICKS_PER_MM_ARM);
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while (opModeIsActive() &&
                (armMotor.isBusy())) {
            printStatus();
        }
        armMotor.setPower(0);
    }

    // TODO find the hold and release position;
    private void hold() {
        setGrabServo(GRABBER_GRAB);
    }

    private void release() {
        setGrabServo(GRABBER_RELEASE);
    }

    private void setGrabServo(double position) {
        grabber.setPosition(position);

    }

    // Drive Using Encoder
    /**
     *
     * @param power = motor power
     * @param leftInches = how many inches you want leftMotor to go
     * @param rightInches = how many inches you want rightMotor to go
     * @param timeoutS = how long the drive should go on for before stopping (just so if we make a mistake it wont go on longer than the timeoutS)
     */
    public void encoderDrive(double power, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            telemetry.addData("rightTarget", newRightTarget);

            leftMotor.setTargetPosition(-newLeftTarget);
            rightMotor.setTargetPosition(-newRightTarget);

            leftMotor.setPower(power);
            rightMotor.setPower(power);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }


    }
    public void whileDrive(double power, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            if(leftInches < 0) {

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
            }
            else {
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
        
    public void whileTurn(double power, double degree) {
        if(opModeIsActive()) {
            int newLeftTarget;
            int newRightTarget;
            if(degree < 0) {

                newLeftTarget = leftMotor.getCurrentPosition() + (int)calcTurn(-degree);
                newRightTarget = rightMotor.getCurrentPosition() + (int)calcTurn(degree);
                leftMotor.setPower(-power);
                rightMotor.setPower(power);

                while (opModeIsActive() && leftMotor.getCurrentPosition() >= newLeftTarget && rightMotor.getCurrentPosition() <= newRightTarget) {
                    telemetry.addData("WhileTurn", 10);
                    telemetry.addData("CurrentLeft: ", leftMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newLeftTarget);
                    telemetry.addData("CurrentRight: ", rightMotor.getCurrentPosition());
                    telemetry.addData("TargetRight: ", newRightTarget);
                    telemetry.update();
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            else {

                newLeftTarget = leftMotor.getCurrentPosition() + (int)calcTurn(degree);
                newRightTarget = rightMotor.getCurrentPosition() + (int)calcTurn(-degree);
                leftMotor.setPower(power);
                rightMotor.setPower(-power);

                while (opModeIsActive() && leftMotor.getCurrentPosition() <= newLeftTarget && rightMotor.getCurrentPosition() >= newRightTarget) {
                    telemetry.addData("WhileTurn", 10);
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

    public void resetEncoder() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurnLeft(double power, double degree, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        resetEncoder();

        if (opModeIsActive()) {
            newLeftTarget = -leftMotor.getCurrentPosition() + (int)calcTurn(degree);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = rightMotor.getCurrentPosition() - (int)calcTurn(degree);
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
                telemetry.addData("Path2",  "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
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
            newLeftTarget = leftMotor.getCurrentPosition() - (int)calcTurn(degree);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = -rightMotor.getCurrentPosition() + (int)calcTurn(degree);
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
                telemetry.addData("Path2",  "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoder();
        }


    }

    private void encoderDrive(double power, double distance, double timeOutS) {
        encoderDrive(power, -distance, -distance, timeOutS);
    }

    // turn using encoder

    /**
     *
     * @param power = motor power
     * @param degree = degree you want to turn by ( - for left, + for right)
     * @param timeoutS = how long the drive should go on for before stopping (just so if we make a mistake it wont go on longer than the timeoutS)
     */
    public void encoderTurn(double power, double degree, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int)calcTurn(-degree);
            telemetry.addData("leftTarget", newLeftTarget);
            newRightTarget = rightMotor.getCurrentPosition() + (int)calcTurn(degree);
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
                telemetry.addData("RightMotor-Current", rightMotor.getCurrentPosition());
                telemetry.addData("RightMotor-Target", newRightTarget);
                telemetry.addData("LeftMotor-Current", leftMotor.getCurrentPosition());
                telemetry.addData("LeftMotor-Target", newLeftTarget);
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    private double calcTurn(double degree) {
        return (Math.PI * ROBOT_WIDTH * (degree / 360)) * COUNTS_PER_INCH;
    }

    // prints the imu status and all motor statuses
    private void printStatus() {
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("armMotor", armMotor.getCurrentPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("foundationLeft", foundationLeft.getPosition());
        telemetry.addData("foundationRight", foundationRight.getPosition());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }




}
