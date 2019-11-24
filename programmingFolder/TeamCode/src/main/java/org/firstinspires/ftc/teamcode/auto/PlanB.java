package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PlanB extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            planB();
        }
        // stop
    }

    public void planB() {
        sleep(20000);
        drive(DRIVE_SPEED, TILE_LENGTH);
        brake();
    }


}
