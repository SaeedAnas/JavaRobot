package org.firstinspires.ftc.teamcode.auto.foundation;

import org.firstinspires.ftc.teamcode.auto.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.Constants.*;

abstract class Foundation extends Autonomous {
    // PLEASE READ:
    // ALWAYS CODE FOR BLUE TEAM AND ADD * turnVal to the turns
    // -Degree is left, +Degree is right

    private static int turnVal;

    private static void checkTeam(char team) {
        if (team == 'b')
            turnVal = 1;
        else if (team == 'r')
            turnVal = -1;
    }

    private void moveFoundationHorizontal(char team) {
        checkTeam(team);
        releaseFoundation();
        drive(-DRIVE_SPEED, -3);
        turnByGyro(TURN_SPEED, -45 * turnVal);
        drive(-DRIVE_SPEED, -30);
        turnByGyro(TURN_SPEED, 45 * turnVal);
        drive(-DRIVE_SPEED, -(TILE_LENGTH - 18));
        grabFoundation();
        sleep(1000);
        drive(DRIVE_SPEED, 5);
        turnByGyro(1, -90 * turnVal);
        releaseFoundation();
        drive(DRIVE_SPEED, TILE_LENGTH - 3);
        brake();
    }

//    private void moveFoundationVertical(char team) {
//        releaseFoundation();
//        drive(-DRIVE_SPEED, -(TILE_LENGTH + 12));
//        grabFoundation();
//        sleep(1000);
//        drive(DRIVE_SPEED, TILE_LENGTH + 7);
//        releaseFoundation();
//        turnByGyro(TURN_SPEED, -100 * turnVal);
//        drive(DRIVE_SPEED, ROBOT_LENGTH);
//        turnByGyro(TURN_SPEED, -100 * turnVal);
//        drive(DRIVE_SPEED, ROBOT_LENGTH + 15);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH);
//        turnByGyro(TURN_SPEED, -95 * turnVal);
//        drive(DRIVE_SPEED, TILE_LENGTH * 2);
//        brake();
//    }

    void currentFoundation(char team) {
        moveFoundationHorizontal(team);
    }


}
