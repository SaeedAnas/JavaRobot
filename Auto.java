
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    // TODO : Measure the Diameter of the Wheels (inches) and input the value here

    private static final double DIAMETER = 4;
    private static final double WHEEL_WIDTH = 2.5;


    // TODO : Measure the Width of the drive train (inches) and input the value here

    private static final double ROBOT_WIDTH = 17.5;

    // One full rotation (degrees)

    private static final int FULL = 360;

    // One full rotation (ticks of motor)

    private static final int FULL_TICKS = 1440;

    private ElapsedTime runtime = new ElapsedTime();

    private static final double
            COUNTS_PER_MOTOR_REV = 1440,
            DRIVE_GEAR_REDUCTION = 2.0, // this is < 1.0 if geared UP
            WHEEL_DIAMETER_INCHES = 4.0,
            COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI),
            DRIVE_SPEED = 0.6,
            TURN_SPEED = 0.5;


    // TODO : attach the drive train motors and the foundation thing

    private DcMotor leftMotor;

    private DcMotor rightMotor;

    private DcMotor foundationGrabber;

    private DcMotor bottomArm;

    private DcMotor middleArm;

    private Servo grabber;

    private Servo foundationRight;

    private Servo foundationLeft;


    // IMU sensors

    private BNO055IMU imu;
    private Orientation OriginAngle = new Orientation();
    private double deltaAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        //Init the hardware


        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        foundationGrabber = hardwareMap.get(DcMotor.class, "grab");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initImu();

        // wait for the run

        waitForStart();

        // loop for the actions


        while (opModeIsActive()) {
            printStatus();

            rotate(90, TURN_SPEED);
        encoderDrive(TURN_SPEED,12,-12,5.0);

        }

        // stop

    }

    // Servo code

    private void setFoundationServo(double position) {
        foundationLeft.setPosition(position);
        foundationRight.setPosition(position);

    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


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


    private void resetAngle()
    {
        initImu();
         OriginAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

    }


    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        deltaAngle = angles.firstAngle - OriginAngle.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        return deltaAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                printStatus(degrees);
            }

            while (opModeIsActive() && getAngle() > degrees) {
                printStatus(degrees);
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                printStatus(degrees);
            }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

    }

    private void printStatus() {
        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Origin", OriginAngle.firstAngle);
        telemetry.addData("DeltaAngle", deltaAngle);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }
    private void printStatus(double degree) {
        telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("degree", degree);
        telemetry.addData("DeltaAngle", deltaAngle);
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }

    private void moveForwardByEncoder(double distance, double power) {
        // calculate distance

        double distanceInTicks = calcDistanceStraight(distance);
        int d = (int) distanceInTicks;

        telemetry.addData("d" , d);
        telemetry.addData("Left", leftMotor.getCurrentPosition());
        telemetry.addData("predictedLeft", leftMotor.getCurrentPosition()+ d);
        telemetry.addData("Right", rightMotor.getCurrentPosition());
        telemetry.addData("predictedRight", rightMotor.getCurrentPosition() + d);

        telemetry.update();


        // set target positions
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() +d);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() +d);
        // set power/direction
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        // make them move
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("LeftAfter", leftMotor.getCurrentPosition());
        telemetry.addData("RightAfter", rightMotor.getCurrentPosition());

        telemetry.update();
    }


    private void moveForwardUsingLoop (double distance, double power) {
        double distanceInTicks = calcDistanceStraight(distance);
        double avg = getAverageEncoder() + distanceInTicks;
        while (rightMotor.getCurrentPosition() < avg && leftMotor.getCurrentPosition() < avg) {
            rightMotor.setPower(1);
            leftMotor.setPower(1);
            avg = getAverageEncoder();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    private double getAverageEncoder() {
        return (rightMotor.getCurrentPosition() + leftMotor.getCurrentPosition());
    }


    private void moveForwardByTime(int time, double power) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        TimeUnit.MILLISECONDS.sleep(time);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void moveBackwardByEncoder (double distance, double power){
        moveForwardByEncoder(distance, -power);
    }

    private void moveBackwardByTime (int time, double power) throws InterruptedException {
        moveForwardByTime(time, -power);
    }

    /**
     * Allows robot to turn at a certain angle
     * @param leftMotor = motor attached to left wheel
     * @param rightMotor = motor attached to right wheel
     * @param power = the power you want the set to the motor (between 0 and 1, negatives are handled in the method)
     * @param direction = a character : 'l' is left, 'r' is right.
     * @param angle = angle in degrees that you want to move (0 - whatever)
     */

    private static void move(DcMotor leftMotor, DcMotor rightMotor, int power, char direction, double angle) {
        if(direction == 'l') {
            // calculate distance
            int distanceInTicks = calcDistanceAngle(angle);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + distanceInTicks);
            // set power/direction
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(direction == 'r'){
            // calculate distance
            int distanceInTicks = calcDistanceAngle(angle);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + distanceInTicks);
            // set power/direction
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }



    /**
     * Calculates distance wheels need to move straight
     * @param distance = distance (inches)
     * @return = returns the calculated distance in ticks
     */

    private static double calcDistanceStraight(double distance) {
        return (distance / (DIAMETER * Math.PI * FULL_TICKS));
    }

    /**
     * Calculates distance wheels need to move left or right
     * @param distance = distance (inches)
     * @return = returns the calculated distance in ticks
     */

    private static int calcDistanceTurn(double distance) {
        return (int) Math.rint(distance /Math.sqrt((DIAMETER * Math.PI)) * FULL_TICKS);
    }

    /**
     *
     * @param angle = angle in degrees (0 - whatever)
     * @return = returns calculated distance in ticks
     */

    private static int calcDistanceAngle(double angle) {
        double TURN_CONSTANT = ((((ROBOT_WIDTH * Math.PI)/4)/(DIAMETER *Math.PI))*FULL_TICKS) / 90;
        return (int) Math.rint(TURN_CONSTANT * (angle/360));
    }

    /**
     *
     * @param degree = angle you want to move (in degrees)
     * @param motor = the motor you want to move
     * @return
     */

    private static int calcDegreeInTicks (double degree, DcMotor motor) {
        double degreesPercent = degree/FULL;
        return (int) Math.rint(motor.getCurrentPosition() + (degreesPercent * FULL_TICKS));
    }

    /**
     *
     * @param degree = double between 0 and 1
     * @param servo = servo you want to move
     */

    private static void moveServo(double degree, Servo servo) {
        if( degree <= 1 || degree >= 0) {
            servo.setPosition(degree);
        }
    }



}
