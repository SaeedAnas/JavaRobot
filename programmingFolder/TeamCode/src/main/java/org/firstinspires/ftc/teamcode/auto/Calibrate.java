package org.firstinspires.ftc.teamcode.auto;

public class Calibrate extends Autonomous {
    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            calibrate();
        }
        // stop
    }

}
