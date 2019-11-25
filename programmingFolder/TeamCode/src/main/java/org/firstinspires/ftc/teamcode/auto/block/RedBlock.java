package org.firstinspires.ftc.teamcode.auto.block;

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
