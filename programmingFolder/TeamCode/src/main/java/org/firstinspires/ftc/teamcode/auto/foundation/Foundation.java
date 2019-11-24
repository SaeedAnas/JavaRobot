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
        encoderTurn(TURN_SPEED, -45 * turnVal, 5.0);
        drive(-DRIVE_SPEED, -30);
        encoderTurn(TURN_SPEED, 45 * turnVal, 5.0);
        drive(-DRIVE_SPEED, -(TILE_LENGTH - 18));
        grabFoundation();
        sleep(1000);
        drive(DRIVE_SPEED, 5);
        encoderTurnOneWheel(1, -90 * turnVal, 5.0);
        releaseFoundation();
        drive(DRIVE_SPEED, TILE_LENGTH - 3);
        brake();
    }

    private void moveFoundationVertical(char team) {
        releaseFoundation();
        drive(-DRIVE_SPEED, -(TILE_LENGTH + 12));
        grabFoundation();
        sleep(1000);
        drive(DRIVE_SPEED, TILE_LENGTH + 7);
        releaseFoundation();
        encoderTurn(TURN_SPEED, -100 * turnVal, 5.0);
        drive(DRIVE_SPEED, ROBOT_LENGTH);
        encoderTurn(TURN_SPEED, -100 * turnVal, 5.0);
        drive(DRIVE_SPEED, ROBOT_LENGTH + 15);
        encoderTurn(TURN_SPEED, -95 * turnVal, 5.0);
        drive(DRIVE_SPEED, TILE_LENGTH);
        encoderTurn(TURN_SPEED, -95 * turnVal, 5.0);
        drive(DRIVE_SPEED, TILE_LENGTH);
        encoderTurn(TURN_SPEED, -95 * turnVal, 5.0);
        drive(DRIVE_SPEED, TILE_LENGTH * 2);
        brake();
    }

    void currentFoundation(char team) {
        moveFoundationHorizontal(team);
    }


}
