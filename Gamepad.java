package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="gamepad", group="Forward")
public class Gamepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftRearMotor = hardwareMap.dcMotor.get("m1");
        DcMotor rightRearMotor = hardwareMap.dcMotor.get("m2");
        DcMotor rightFrontMotor = hardwareMap.dcMotor.get("m3");
        DcMotor leftFrontMotor = hardwareMap.dcMotor.get("m4");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Encoder Value: ", (rightFrontMotor.getCurrentPosition()
                    +leftFrontMotor.getCurrentPosition()+leftRearMotor.getCurrentPosition()
                    +rightRearMotor.getCurrentPosition())/4);
            if(gamepad1.right_trigger > 0){ //forward
                rightFrontMotor.setPower(gamepad1.right_trigger);
                leftFrontMotor.setPower(gamepad1.right_trigger);
                rightRearMotor.setPower(gamepad1.right_trigger);
                leftRearMotor.setPower(gamepad1.right_trigger);
                telemetry.update();
            }
            else if(gamepad1.left_trigger > 0){ //backward
                rightFrontMotor.setPower(-gamepad1.left_trigger);
                leftFrontMotor.setPower(-gamepad1.left_trigger);
                rightRearMotor.setPower(-gamepad1.left_trigger);
                leftRearMotor.setPower(-gamepad1.left_trigger);
            }
            else if(gamepad1.right_bumper){ //strafe right
                rightFrontMotor.setPower(-1);
                leftFrontMotor.setPower(1);
                rightRearMotor.setPower(1);
                leftRearMotor.setPower(-1);
            }
            else if(gamepad1.left_bumper){ //strafe left
                rightFrontMotor.setPower(1);
                leftFrontMotor.setPower(-1);
                rightRearMotor.setPower(-1);
                leftRearMotor.setPower(1);
            }
            else if(gamepad1.dpad_left){ //turn left
                rightFrontMotor.setPower(1);
                leftFrontMotor.setPower(-1);
                rightRearMotor.setPower(1);
                leftRearMotor.setPower(-1);
            }
            else if(gamepad1.dpad_right){ //turn right
                rightFrontMotor.setPower(-1);
                leftFrontMotor.setPower(1);
                rightRearMotor.setPower(-1);
                leftRearMotor.setPower(1);
            }
            else{ //stop
                rightFrontMotor.setPower(0);
                leftFrontMotor.setPower(0);
                rightRearMotor.setPower(0);
                leftRearMotor.setPower(0);
            }
        }

    }
}
