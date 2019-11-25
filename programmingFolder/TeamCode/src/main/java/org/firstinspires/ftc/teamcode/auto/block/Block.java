package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.Constants.*;

abstract class Block extends Autonomous {
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
    private void grabBlock(char team) {
        checkTeam(team);
        release();
        whileArm(20, ARM_SPEED);
        sleep(500);
        drive(DRIVE_SPEED, (TILE_LENGTH * 2) - ROBOT_LENGTH);
        sleep(500);
        whileArm(-10, ARM_SPEED + 0.1);
        sleep(500);
        grab();
        sleep(500);
        drive(-DRIVE_SPEED, -((TILE_LENGTH * 2) - ROBOT_LENGTH));
        turnByGyro(0.2, -95 * turnVal);
        drive(DRIVE_SPEED, 10);
        release();
    }

     void currentBlock(char team) {
        grabBlock(team);
    }
}
