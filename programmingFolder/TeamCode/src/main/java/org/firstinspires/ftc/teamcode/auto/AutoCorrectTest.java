package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoCorrectTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            autoCorrectMove(FORWARD, 50, 1);
            sleep(1000);
            autoCorrectMove(BACKWARD, 50, 1);

        }
        // stop
    }

}
