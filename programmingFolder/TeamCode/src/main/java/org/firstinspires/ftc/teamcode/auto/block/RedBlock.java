package org.firstinspires.ftc.teamcode.auto.block;

import org.firstinspires.ftc.teamcode.auto.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedBlock extends Block {
    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            currentBlock('r');
        }
        // stop
    }
}
