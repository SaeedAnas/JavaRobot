package org.firstinspires.ftc.teamcode.auto;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class ImuTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        while (opModeIsActive()) {
            // Left
            turnByGyro(0.3,-90);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
            // Right
            turnByGyro(0.3,90);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
            // Left
            turnByGyro(0.3,-45);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
            // Right
            turnByGyro(0.3,45);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
            // Left
            turnByGyro(0.3,-180);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
            // Right
            turnByGyro(0.3,180);
            telemetry.addData("Out of method", 6);
            telemetry.update();
            sleep(1000);
        }
        // stop
    }

}
// 179 -> -179
// y value
// right decreasing
// left increasing