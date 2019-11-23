
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
// Docs
// https://ftctechnh.github.io/ftc_app/doc/javadoc/org/firstinspires/ftc/robotcore/external/navigation/package-summary.html

@Autonomous
public class Auto extends LinearOpMode {

    // WARNING : CODE HAS NOT BEEN TESTED AND MAY NOT WORK

    // runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Encoder variables
    private static final double
            ROBOT_WIDTH = 16.5,
            ROBOT_LENGTH = 15.25,
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
            TICKS_PER_MM_ARM = (GEAR_RATIO_ARM * COUNTS_PER_MOTOR_REV)/P_DISTANCE_PER_ROTATION
            //TICKS_PER_INCH_ARM = TICKS_PER_MM_ARM / 25.4,
            //BLOCK_HEIGHT_MM = 101.6,
            //BLOCK_HEIGHT_INCHES = BLOCK_HEIGHT_MM / 25.4,
            //TIME_FOR_HORIZONTAL_ARM = 7
            ;



    // Hardware
    private DcMotor leftMotor;

    private DcMotor rightMotor;

    private DcMotor armMotorLeft;

    private DcMotor armMotorRight;

    private Servo grabber;

    private Servo foundationRight;

    private Servo foundationLeft;

    private CRServo armServo;

    private BNO055IMU imu;

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);



    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        armMotorLeft = hardwareMap.get(DcMotor.class,"armLeft");
        armMotorRight = hardwareMap.get(DcMotor.class, "armRight");
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
            foundation2();
        }
        // stop
    }

    private void foundation2 () {
        // make sure the foundation isnt in the way
        releaseFoundation();
        // go back
        whileDrive(-DRIVE_SPEED, -((TILE_LENGTH*2) - ROBOT_LENGTH));
        // grab the foundation
        grabFoundation();
        // drive straight
        whileDrive(DRIVE_SPEED,TILE_LENGTH);
        // turn the foundation left
        encoderTurnLeft(TURN_SPEED, 110, 5.0);
        // go back into the area
        whileDrive(-DRIVE_SPEED, -30);
        // release the foundation
        releaseFoundation();
        // drive under the bridge
        whileDrive(DRIVE_SPEED, TILE_LENGTH*2);
    }
    private void red() {
        // back: 31 inches
        // front:
        releaseFoundation();
        // go back
        whileDrive(-DRIVE_SPEED, -((TILE_LENGTH*2) - ROBOT_LENGTH));
        // get foundation
        grabFoundation();
        // go straight
        whileDrive(DRIVE_SPEED,TILE_LENGTH);
        // release
        releaseFoundation();
        // turn left
        encoderTurnRight(TURN_SPEED, 100, 5.0);
        // go out of the foundation
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        // turn left
        encoderTurnRight(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2.5);
        // turn left
        encoderTurnRight(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        encoderTurnRight(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH);
        encoderTurnRight(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void blue() {
        // back: 31 inches
        // front:
        releaseFoundation();
        // go back
        whileDrive(-DRIVE_SPEED, -((TILE_LENGTH*2) - ROBOT_LENGTH));
        // get foundation
        grabFoundation();
        // go straight
        whileDrive(DRIVE_SPEED,TILE_LENGTH);
        // release
        releaseFoundation();
        // turn left
        encoderTurnLeft(TURN_SPEED, 100, 5.0);
        // go out of the foundation
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        // turn left
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2.5);
        // turn left
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH);
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void block() {

    whileDrive(DRIVE_SPEED, TILE_LENGTH * 3);

    whileDrive(DRIVE_SPEED, 90);

    }

    private void foundation(int team) {
        // back: 31 inches
        // front:
        releaseFoundation();
        // go back
        whileDrive(-DRIVE_SPEED, -((TILE_LENGTH*2) - ROBOT_LENGTH));
        // get foundation
        grabFoundation();
        // go straight
        whileDrive(DRIVE_SPEED,TILE_LENGTH);
        // release
        releaseFoundation();
        // turn left
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        // go out of the foundation
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        // turn left
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2.5);
        // turn left
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        // go forward
        whileDrive(DRIVE_SPEED, ROBOT_LENGTH);
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH);
        encoderTurnLeft(TURN_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH * 2);

    }

    // if all else fails...

    private void planB() {
        sleep(20000);
        encoderTurnLeft(DRIVE_SPEED, 90, 5.0);
        whileDrive(DRIVE_SPEED, TILE_LENGTH);
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
        int target = armMotorLeft.getCurrentPosition() + (int)(mm * TICKS_PER_MM_ARM);
        armMotorLeft.setTargetPosition(target);
        armMotorRight.setTargetPosition(target);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        while (opModeIsActive() &&
                (armMotorLeft.isBusy() && armMotorRight.isBusy())) {
            printStatus();
        }
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
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

    private void whileDrive(double power, double distance) {
        whileDrive(power, distance, distance);

    }

    // Drive Using Encoder
    private void whileDrive(double power, double leftInches, double rightInches) {
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
//        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).secondAngle);
//        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("Z",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("armMotorLeft", armMotorLeft.getCurrentPosition());
        telemetry.addData("armMotorRight", armMotorRight.getCurrentPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("foundationLeft", foundationLeft.getPosition());
        telemetry.addData("foundationRight", foundationRight.getPosition());
        telemetry.addData("ArmServo", armServo.getPower());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    private double getGyroXAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return (angles.firstAngle);
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

    private void turnLeftByGyro (double motorPower, int targetDegree) {
        double zeroReference = getGyroXAngle();
        double angleTurned = 0;
        rightMotor.setPower(motorPower);
        leftMotor.setPower(-motorPower);
        while( opModeIsActive() && (angleTurned < targetDegree))
        {
            double currentAngle = getGyroXAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    private void turnRightByGyro (double motorPower, int targetDegree) {
        double zeroReference = getGyroXAngle();
        double angleTurned = 0;
        rightMotor.setPower(motorPower);
        leftMotor.setPower(-motorPower);
        while( opModeIsActive() && (angleTurned > (-targetDegree)))
        {
            double currentAngle = getGyroXAngle();
            angleTurned = adjustedAngle(zeroReference, currentAngle);
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

//    private void initImu() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//    }
//
//    private double getAngle()
//    {
//        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
//    }
//
//    private void rotate(double power, int degrees)
//    {
//
//        if(degrees == 0) {
//            return;
//        }
//        // degrees < 0 for left, degrees > 0 for right
//        double targetDegree = getAngle() + degrees;
//
//        // if the target degree < 180, we add 360 and make it positive
//        // ex. getAngle() = -100, degrees = -90, -> targetDegree = -190, but the imu will not go to -190,
//        // but instead become 180. In this case, we would want the imu to stop when the degree is 170, so -190 + 360 = 170!
//        // go left
//        if (targetDegree < -180) {
//            // targetDegree = -190 -> 170
//            // current = -100
//            targetDegree += 360;
//            leftMotor.setPower(-power);
//            rightMotor.setPower(power);
//            // first wait until the angle has reached -180 and become 180
//            while(opModeIsActive() && (getAngle() < 0)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//            // then wait until the current angle becomes less than the targetDegree
//            // ex. if targetDegree is 170, then it will stop there.
//            while(opModeIsActive() && (getAngle() > targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//
//        }
//        // if the target degree > 180, we subtract 360 and make it negative
//        // ex. getAngle() = 100, degrees = 90, -> targetDegree = 190, but the imu will not go to 190,
//        // but instead become -180. In this case, we would want the imu to stop when the degree is -170, so 190 - 360 = -170!
//        // go right
//        else if (targetDegree > 180) {
//
//            targetDegree -= 360;
//            leftMotor.setPower(power);
//            rightMotor.setPower(-power);
//            // first wait until the current angle reaches 180 and goes to -180
//            while(opModeIsActive() && (getAngle() > 0)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//            // then wait until the current angle becomes more than the target degree
//            // ex. if targetDegree is -170, then the robot will stop after the angle is greater than -170
//            while(opModeIsActive() && (getAngle() < targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//
//        }
//        // go left
//        else if (degrees < 0) {
//            leftMotor.setPower(-power);
//            rightMotor.setPower(power);
//            while(opModeIsActive() && (getAngle() > targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//
//        }
//        // go right
//        else {
//            leftMotor.setPower(power);
//            rightMotor.setPower(-power);
//            while(opModeIsActive() && (getAngle() < targetDegree)){
//                telemetry.addData("+", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngle());
//                printStatus();
//            }
//        }
//
//        rightMotor.setPower(0);
//        leftMotor.setPower(0);
//
//    }
//    private void rotateX(double power, int degrees)
//    {
//
//        if(degrees == 0) {
//            return;
//        }
//        // degrees < 0 for left, degrees > 0 for right
//        double targetDegree = getAngleX() + degrees;
//
//        // if the target degree < 180, we add 360 and make it positive
//        // ex. getAngle() = -100, degrees = -90, -> targetDegree = -190, but the imu will not go to -190,
//        // but instead become 180. In this case, we would want the imu to stop when the degree is 170, so -190 + 360 = 170!
//        // go left
//        if (targetDegree < -180) {
//            // targetDegree = -190 -> 170
//            // current = -100
//            targetDegree += 360;
//            leftMotor.setPower(-power);
//            rightMotor.setPower(power);
//            // first wait until the angle has reached -180 and become 180
//            while(opModeIsActive() && (getAngleX() < 0)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//            // then wait until the current angle becomes less than the targetDegree
//            // ex. if targetDegree is 170, then it will stop there.
//            while(opModeIsActive() && (getAngleX() > targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//
//        }
//        // if the target degree > 180, we subtract 360 and make it negative
//        // ex. getAngle() = 100, degrees = 90, -> targetDegree = 190, but the imu will not go to 190,
//        // but instead become -180. In this case, we would want the imu to stop when the degree is -170, so 190 - 360 = -170!
//        // go right
//        else if (targetDegree > 180) {
//
//            targetDegree -= 360;
//            leftMotor.setPower(power);
//            rightMotor.setPower(-power);
//            // first wait until the current angle reaches 180 and goes to -180
//            while(opModeIsActive() && (getAngleX() > 0)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//            // then wait until the current angle becomes more than the target degree
//            // ex. if targetDegree is -170, then the robot will stop after the angle is greater than -170
//            while(opModeIsActive() && (getAngleX() < targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//
//        }
//        // go left
//        else if (degrees < 0) {
//            leftMotor.setPower(-power);
//            rightMotor.setPower(power);
//            while(opModeIsActive() && (getAngleX() > targetDegree)){
//                telemetry.addData("negative", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//
//        }
//        // go right
//        else {
//            leftMotor.setPower(power);
//            rightMotor.setPower(-power);
//            while(opModeIsActive() && (getAngleX() < targetDegree)){
//                telemetry.addData("+", 0);
//                telemetry.addData("targetDegree", targetDegree);
//                telemetry.addData("angle", getAngleX());
//                printStatus();
//            }
//        }
//
//        rightMotor.setPower(0);
//        leftMotor.setPower(0);
//
//    }
//    private double getAngleX()
//    {
//        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
//    }
//
//    private void gyroRight(double power, int degrees) {
//        rotate(power, degrees);
//    }
//    private void gyroRightX(double power, int degrees) {
//        rotateX(power, degrees);
//    }
//
//    private void gyroLeft(double power, int degrees) {
//        rotate(power, -degrees );
//    }
//    private void gyroLeftX(double power, int degrees) {
//        rotate(power, -degrees );
//    }
//
//    private double getGyroXAngle() {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        return angles.firstAngle;
//    }
//    private double adjustedAngle(double zeroReference, double currentAngle) {
//        double adjusted = currentAngle - zeroReference;
//        if (adjusted < -179) {
//            adjusted += 360;
//        }
//        else if (adjusted > 180) {
//            adjusted -=360;
//        }
//        return adjusted;
//    }
//
//    private void turnLeftByGyro(double motorPower, int targetDegree) {
//        double zeroReference = getGyroXAngle();
//        double angleTurned = 0;
//        leftMotor.setPower(-motorPower);
//        rightMotor.setPower(motorPower);
//        while(opModeIsActive() && (angleTurned < targetDegree)) {
//            double currentAngle = getGyroXAngle();
//            angleTurned = adjustedAngle(zeroReference, currentAngle);
//        }
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    private void turnRightByGyro(double motorPower, int targetDegree) {
//        double zeroReference = getGyroXAngle();
//        double angleTurned = 0;
//        leftMotor.setPower(motorPower);
//        rightMotor.setPower(-motorPower);
//        while(opModeIsActive() && (angleTurned > (-targetDegree))){
//            double currentAngle = getGyroXAngle();
//            angleTurned = adjustedAngle(zeroReference, currentAngle);
//        }
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    private double getXAngle() {
//        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
//    }
//
//    private void turnRightGyro(double power, int degree) {
//        double targetDegree = calcAngle(-degree);
//        leftMotor.setPower(power);
//        rightMotor.setPower(-power);
//        double angle = getXAngle();
//        if(targetDegree > 0) {
//            while (opModeIsActive() && (angle < 0)) {
//                printAngle(targetDegree, angle);
//                angle=getXAngle();
//            }
//            while(opModeIsActive() && (targetDegree < angle)) {
//                printAngle(targetDegree, angle);
//                angle=getXAngle();
//            }
//        }
//        else {
//            while (opModeIsActive() && (targetDegree < angle)) {
//                printAngle(targetDegree, angle);
//                angle = getXAngle();
//            }
//        }
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//    private void turnLeftGyro(double power, int degree) {
//        double targetDegree = calcAngle(degree);
//        leftMotor.setPower(-power);
//        rightMotor.setPower(power);
//        double angle = getXAngle();
//        if(targetDegree < 0) {
//            while (opModeIsActive() && (angle > 0)) {
//                printAngle(targetDegree, angle);
//                angle=getXAngle();
//            }
//            while(opModeIsActive() && (targetDegree > angle)) {
//                printAngle(targetDegree, angle);
//                angle=getXAngle();
//            }
//        } else {
//            while (opModeIsActive() && (targetDegree > angle)) {
//                printAngle(targetDegree, angle);
//                angle = getXAngle();
//            }
//        }
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    private double calcAngle(int degree){
//        double targetDegree = getXAngle() - degree;
//        if (targetDegree < -179) {
//            targetDegree += 360;
//        }
//        else if (targetDegree > 180) {
//            targetDegree -= 360;
//        }
//        return targetDegree;
//    }
//
//    private void printAngle(double targetDegree, double angle) {
//        telemetry.addData("X: ", angle);
//        telemetry.addData("Target: ", targetDegree);
//        telemetry.update();
//    }
//




}
