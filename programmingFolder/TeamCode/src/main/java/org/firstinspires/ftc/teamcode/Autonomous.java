package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {

    @Override
    public synchronized void waitForStart() {
        super.waitForStart();
    }

    double WHEEL_DIAMETER = 4;
    double TICKS_PER_REV = 1120;
    double TICKS_TO_MOVE = 0;

    private void distance_formula( double x1, double y1, double x2, double y2, char direction){
        /*
        @param: tiles
        x1 = current x value of the robot
        y1 = current y value of the robot
        x2 = new x value of the robot
        y2 = new y value of the robot
        direction = forward, backward, left, right
         */
            double x_value = (Math.pow(x2 - x1, 2));
            double y_value = (Math.pow(y2 - y1, 2));
            double distance = Math.sqrt(x_value + y_value);


            if (direction == 'f' || direction == 'b') {
                TICKS_TO_MOVE = Math.rint(distance / (WHEEL_DIAMETER * Math.PI)) * TICKS_PER_REV);
            } else if (direction == 'l' || direction == 'r') {
                TICKS_TO_MOVE = Math.rint(distance / Math.sqrt((WHEEL_DIAMETER * Math.PI)) * TICKS_PER_REV);
            }
        }

    private void turn(double angle, char t){
        /*
        @param: angle
        angle = angle the robot should turn to
        t = direction
         */
        double TRACK_WIDTH = 17.5;
        double TURN_CONSTANT = ((((TRACK_WIDTH * Math.PI)/4)/(WHEEL_DIAMETER*Math.PI))*TICKS_PER_REV) / 90;
        double TICKS_TO_MOVE = Math.rint(TURN_CONSTANT * angle);

        if (t == 'l'){
            //right motor moves +TICKS_TO_MOVE
            //left motor moves -TICKS_TO_MOVE

        }

        else if (t == 'r'){
            //right motor moves -TICKS_TO_MOVE
            //left motor moves +TICKS_TO_MOVE
        }

    }

    





}
