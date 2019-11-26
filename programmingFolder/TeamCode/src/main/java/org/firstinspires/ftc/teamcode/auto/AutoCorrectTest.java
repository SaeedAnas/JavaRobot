package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoCorrectTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            autoCorrectingDrive(DRIVE_SPEED, 25);
            sleep(1000);
            autoCorrectingDrive(DRIVE_SPEED, -25);
        }
        // stop
    }

}
