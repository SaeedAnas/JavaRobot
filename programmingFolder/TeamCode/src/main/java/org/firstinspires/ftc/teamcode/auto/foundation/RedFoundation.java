package org.firstinspires.ftc.teamcode.auto.foundation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedFoundation extends Foundation {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            currentFoundation('r');
        }
        // stop
    }

}
