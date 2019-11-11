
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

    private static final double ROBOT_WIDTH = 15.7;

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
        int count = 2;
        while (opModeIsActive()) {
             printStatus();
            releaseFoundation();
            whileDrive(DRIVE_SPEED, 30,30, 5.0);
            grabFoundation();
            whileDrive(DRIVE_SPEED, -30, -30, 5.0);
            releaseFoundation();
            encoderTurn(TURN_SPEED, 90,5.0);
            encoderTurn(TURN_SPEED, -90, 5.0);
            encoderTurn(TURN_SPEED, 180, 5.0);
            encoderTurn(TURN_SPEED, -180, 5.0);

        }
        // stop
    }



    private void moveArmServo(double power) {
        armServo.setPower(power);
    }

//    private void moveArmVertical(double mm, double power){
//        int ticksToMove =armMotor.getCurrentPosition() + (int)(mm * TICKS_PER_MM_ARM);
//        armMotor.setTargetPosition(ticksToMove);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(power);
//        while() {
//
//        }
//
//
//    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180){
            deltaAngle += 360;

        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private double checkDirection() {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0){
            correction = 0;

        }
        else {
            correction = -angle;
        }
        correction = correction * gain;
        return correction;
    }

    private void rotate(int degrees, double power) {
        double leftPower, rightPower;
        resetAngle();

        if (degrees < 0) {
            leftPower = power;
            rightPower = -power;

        }
        else if (degrees > 0) {
            leftPower = -power;
            rightPower = power;
        }
        else return;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}


        }
        else {
            while (opModeIsActive() && getAngle() < degrees) {}

            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(1000);
            resetAngle();
        }
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

    private void getBlock(double degreeDown, double degreeUp, double power) {
        moveArmMotor(degreeDown, power);
        hold();
        moveArmMotor(degreeUp, power);
    }

    private void placeBlock(double degreeDown, double degreeUp, double power){
        moveArmMotor(degreeDown,power);
        release();
        moveArmMotor(degreeUp,power);
    }

    private void moveArmMotor(double mm, double power) {
        int target = armMotor.getCurrentPosition() + (int)(mm * TICKS_PER_MM_ARM);
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while (opModeIsActive() &&
                (armMotor.isBusy())) {

        }
        armMotor.setPower(0);
    }

    // TODO find the hold and release position;
    private void hold() {
        setGrabServo(1);
    }

    private void release() {
        setGrabServo(0);
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
                leftMotor.setPower(-power);
                rightMotor.setPower(-power);

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
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
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

    // initializes the IMU
    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }


    // Only use at the beginning of rotate - it resets the imu and gets the origin value
//    private void resetAngle1()
//    {
//         initImu();
//         // get the origin
//         OriginAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
//
//
//    }

    // returns the current angle
    private double getAngle1()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
    }


//
//    // turns robot using the imu
//    private void rotate1(double degrees, double power)
//    {
//        double  leftPower, rightPower;
//
//        // restart imu movement tracking.
//        resetAngle();
//
//        // this is the target degree
//        degrees += OriginAngle.firstAngle;
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // turn right when degrees is less than 0
//        if (degrees < 0)
//        {   // turn right.
//            leftPower = power;
//            rightPower = -power;
//        }
//
//        // turn left when degrees is greater than 0
//        else if (degrees > 0)
//        {   // turn left.
//            leftPower = -power;
//            rightPower = power;
//        }
//        else return;
//
//        // set power to rotate.
//        leftMotor.setPower(leftPower);
//        rightMotor.setPower(rightPower);
//
//        // rotate until turn is completed.
//        // right turn
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() > degrees) {
//                printStatus(degrees);
//            }
//        }
//        // left turn
//        else {
//
//            while (opModeIsActive() && getAngle() < degrees) {
//                printStatus(degrees);
//            }
//        }
//
//        // turn the motors off.
//        rightMotor.setPower(0);
//        leftMotor.setPower(0);
//
//        // wait for rotation to stop.
//        sleep(1000);
//
//    }

    // prints the imu status and all motor statuses
    private void printStatus() {
//        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//        telemetry.addData("Origin", OriginAngle.firstAngle);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("armMotor", armMotor.getCurrentPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("foundationLeft", foundationLeft.getPosition());
        telemetry.addData("foundationRight", foundationRight.getPosition());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }
    private void printStatus(double degree) {
//        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//        telemetry.addData("Origin", OriginAngle.firstAngle);
        telemetry.addData("degree", degree);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("armMotor", armMotor.getCurrentPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("foundationLeft", foundationLeft.getPosition());
        telemetry.addData("foundationRight", foundationRight.getPosition());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }

    // move using time
    private void moveForwardByTime(int time, double power) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        TimeUnit.MILLISECONDS.sleep(time);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void moveBackwardByTime (int time, double power) throws InterruptedException {
        moveForwardByTime(time, -power);
    }

    private void turnRightByTime(int time, double power) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
        TimeUnit.MILLISECONDS.sleep(time);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void turnLefttByTime(int time, double power) throws InterruptedException {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        TimeUnit.MILLISECONDS.sleep(time);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


}
