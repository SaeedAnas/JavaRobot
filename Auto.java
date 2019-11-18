
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    // runtime
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();

    // Encoder variables
    private static final double
            ROBOT_WIDTH = 18,
            ROBOT_LENGTH = 18,
            TILE_LENGTH = 23.75,
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 0.5, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 4.0,
            COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            DRIVE_SPEED = 0.4,
            TURN_SPEED = 0.3,
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
            BLOCK_HEIGHT_INCHES = BLOCK_HEIGHT_MM / 25.4,
            TIME_FOR_HORIZONTAL_ARM = 7;



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
    private Orientation OriginAngle = new Orientation();
    private double deltaAngle;
    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init the hardware
        initImu();
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

        // wait for play
        waitForStart();

        // run
        while (opModeIsActive()) {
            gyroLeft(TURN_SPEED,90);
            sleep(2000);
            gyroRight(TURN_SPEED,90);
            sleep(2000);
            gyroLeft(TURN_SPEED,45);
            sleep(2000);
            gyroRight(TURN_SPEED,45);
            sleep(2000);

        }
        // stop
    }

    private void red() {
        foundation(1);
    }

    private void blue() {
        foundation(-1);
    }

    private void block() {
    encoderTurnRight(TURN_SPEED, 90, 5.0);
    whileDrive(DRIVE_SPEED, TILE_LENGTH * 3, 5.0);
    encoderTurnLeft(TURN_SPEED, 90, 5.0);
    whileDrive(DRIVE_SPEED, 90, 5.0);

    }

    private void foundation(int team) {
        releaseFoundation();
        whileDrive(-DRIVE_SPEED, -((TILE_LENGTH*2) - ROBOT_LENGTH), 5.0);
        grabFoundation();
        whileDrive(DRIVE_SPEED,50,5.0);
        releaseFoundation();
        gyroLeft(TURN_SPEED, 90 * team);
        whileDrive(DRIVE_SPEED, TILE_LENGTH, 5.0);
        gyroLeft(TURN_SPEED, 90 * team);
        whileDrive(DRIVE_SPEED, 18.5 + ROBOT_LENGTH, 5.0);
        gyroLeft(TURN_SPEED, 90 * team);
        whileDrive(DRIVE_SPEED, 34.25/2, 5.0);
        gyroLeft(TURN_SPEED, 90 * team);
        whileDrive(DRIVE_SPEED, TILE_LENGTH, 5.0);
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

    public void whileDrive(double power, double distance, double timeoutS) {
        whileDrive(power, distance,distance, timeoutS);

    }

    // Drive Using Encoder
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

    private double calcTurn(double degree) {
        return (Math.PI * ROBOT_WIDTH * (degree / 360)) * COUNTS_PER_INCH;
    }

    // prints the imu status and all motor statuses
    private void printStatus() {
        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Z",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("armMotor", armMotor.getCurrentPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("foundationLeft", foundationLeft.getPosition());
        telemetry.addData("foundationRight", foundationRight.getPosition());
        telemetry.addData("ArmServo", armServo.getPower());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    private double getAngle()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
    }

    private void rotate(double power, int degrees)
    {

        if(degrees == 0) {
            return;
        }

        double targetDegree = getAngle() + degrees;

        // if the target degree < 180, we add 360 and make it positive
        // ex. getAngle() = -100, degrees = -90, -> targetDegree = -190, but the imu will not go to -190,
        // but instead become 180. In this case, we would want the imu to stop when the degree is 170, so -190 + 360 = 170!
        if (targetDegree < -180) {

            targetDegree = (targetDegree + 360);
            // TODO right turn - if this doesn't work the try replacing this with left turn
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            // current = -100 -> 260, target = 170, degree is < 0, so it will turn right;
            while(opModeIsActive() && (getAngle() + 360 > targetDegree)){
                telemetry.addData("negative", 0);
                telemetry.addData("targetDegree", targetDegree);
                telemetry.addData("angle", getAngle());
                printStatus();
            }

        }
        // if the target degree > 180, we subtract 360 and make it negative
        // ex. getAngle() = 100, degrees = 90, -> targetDegree = 190, but the imu will not go to 190,
        // but instead become -180. In this case, we would want the imu to stop when the degree is -170, so 190 - 360 = -170!
        else if (targetDegree > 180) {

            targetDegree = (targetDegree - 360);
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            // current = 100 -> -260, target = -170, degree > 0, so it will turn left;
            while(opModeIsActive() && (getAngle() - 360 < targetDegree)){
                telemetry.addData("negative", 0);
                telemetry.addData("targetDegree", targetDegree);
                telemetry.addData("angle", getAngle());
                printStatus();
            }

        }

        else if (degrees < 0) {
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            while(opModeIsActive() && (getAngle() > targetDegree)){
                telemetry.addData("negative", 0);
                telemetry.addData("targetDegree", targetDegree);
                telemetry.addData("angle", getAngle());
                printStatus();
            }

        } else {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            while(opModeIsActive() && (getAngle() < targetDegree)){
                telemetry.addData("+", 0);
                telemetry.addData("targetDegree", targetDegree);
                telemetry.addData("angle", getAngle());
                printStatus();
            }
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);

    }

    private void gyroRight(double power, int degrees) {
        rotate(power, -degrees);
    }

    private void gyroLeft(double power, int degrees) {
        rotate(power, degrees );
    }




}
