package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


@Autonomous
public class SampleAutoOp extends LinearOpMode {

    // WARNING : CODE HAS NOT BEEN TESTED AND MAY NOT WORK
    
    // TODO : Measure the Diameter of the Wheels (inches) and input the value here
    
    private static final double DIAMETER = 4.2;
    
    // TODO : Measure the Width of the drive train (inches) and input the value here
    
    private static final double ROBOT_WIDTH = 17.5;
    
    // One full rotation (degrees)
    
    private static final int FULL = 360;
    
    // One full rotation (ticks of motor)

    private static final int FULL_TICKS = 1440;
    
    // TODO : attach the drive train motors and the foundation thing

    private DcMotor leftMotor;

    private DcMotor rightMotor;
    
    private DcMotor foundationGrabber;

    private Servo left, right;


    /**
     *
     * @param degree = angle you want to move (in degrees)
     * @param motor = motor you want to move
     * @param power = power you want (-1 to 1)
     */

    private static void rotateByDegree(int degree, DcMotor motor, int power) {
        int distanceInTicks = calcDegreeInTicks(degree, motor);
        // set target positions
        motor.setTargetPosition(motor.getCurrentPosition() + distanceInTicks);
        // set power/direction
        motor.setPower(power);
        // make them move
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     *
     * @param distance = the distance you want to travel (in inches)
     * @param leftMotor = the motor attached to the left wheel
     * @param rightMotor = the motor attached to the right wheel
     * @param power = the power you want the set to the motor (between 0 and 1, negatives are handled in the method)
     * @param direction = it is a character: 'f' is forward, 'b' is backward, 'r' is right, 'l' is left
     */

    private static void move(double distance, DcMotor leftMotor, DcMotor rightMotor, int power, char direction) {
        if (direction == 'f') {
            // calculate distance
            int distanceInTicks = calcDistanceStraight(distance);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() +distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() +distanceInTicks);
            // set power/direction
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
        else if (direction == 'b') {
            // calculate distance
            int distanceInTicks = calcDistanceStraight(distance);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() +distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() +distanceInTicks);
            // set power/direction
            leftMotor.setPower(-power);
            rightMotor.setPower(-power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if (direction == 'r') {
            // calculate distance
            int distanceInTicks = calcDistanceTurn(distance);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() +distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() +distanceInTicks);
            // set power/direction
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if (direction == 'l') {
            // calculate distance
            int distanceInTicks = calcDistanceTurn(distance);
            // set target positions
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition() +distanceInTicks);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition() +distanceInTicks);
            // set power/direction
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            // make them move
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
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

    private static int calcDistanceStraight(double distance) {
        return (int) Math.rint(distance / (DIAMETER * Math.PI * FULL_TICKS));
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







    @Override
    public void runOpMode() throws InterruptedException {
        //Init the hardware

        // TODO : Make sure the device names are right here

        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
     foundationGrabber = hardwareMap.get(DcMotor.class, "grab");
//        left = hardwareMap.get(Servo.class ,"leftServo");
//        right = hardwareMap.get(Servo.class, "rightServo");

        // wait for the run

        waitForStart();

        // loop for the actions

        int count = 0;

        while (opModeIsActive()) {

            //Code to grab the foundation

            // TODO : Put in measurements for the distance to the foundation and back on the actual track

            int calculatedDistanceTo = 30;
            int calculatedDistanceFrom = 30;

            telemetry.addData("leftMotor Encoder Value:", leftMotor.getCurrentPosition());
            telemetry.addData("rightMotor Encoder Value:", rightMotor.getCurrentPosition());
            telemetry.addData("foundationGrabber Encoder Value:", foundationGrabber.getCurrentPosition());
            telemetry.update();

            // the code only runs once

//            if(count < 1) {
//                move(calculatedDistanceTo, leftMotor, rightMotor, 1, 'f');
//                rotateByDegree(90, foundationGrabber, 1);
//                move(leftMotor, rightMotor, 1, 'r', 180);
//                move(calculatedDistanceFrom, leftMotor, rightMotor, 1, 'f');
//                rotateByDegree(90, foundationGrabber, -1);
//                count++;
//            }

            rightMotor.setPower(1);
            leftMotor.setPower(1);

            TimeUnit.SECONDS.sleep(5);

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            TimeUnit.SECONDS.sleep(5);


            /*

            // Code to make robot go in a square idk


            move(30, leftMotor, rightMotor, 1, 'f');
            move(leftMotor, rightMotor, 1, 'r');

            move(30, leftMotor, rightMotor, 1, 'f');
            move(leftMotor, rightMotor, 1, 'r');

            move(30, leftMotor, rightMotor, 1, 'f');
            move(leftMotor, rightMotor, 1, 'r');

            move(30, leftMotor, rightMotor, 1, 'f');
            move(leftMotor, rightMotor, 1, 'r');


             */





        }

        // stop

    }


}
