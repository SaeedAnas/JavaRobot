package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SampleAutoOp extends LinearOpMode {

    public Hardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        //Init the hardware

        hardware = new Hardware(hardwareMap);

        // wait for the run

        waitForStart();

        // loop for the actions

        while (opModeIsActive()) {
            hardware.setDrivePower(1);

        }




        // stop

    }
}
