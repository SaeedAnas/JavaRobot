package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp (name="Teleop", group="teloop")
public class Teleop extends LinearOpMode {
    //Hardware
    private DcMotor left, right, arm;
    private Servo rightFoundation, leftFoundation, grabber;
    private CRServo armServo;
    //Constants
    private static final double
            LEFT_FOUNDATION_DOWN = 0.4,
            LEFT_FOUNDATION_UP = 0.9,
            RIGHT_FOUNDATION_DOWN = 0.6,
            RIGHT_FOUNDATION_UP = 0.1,
            SERVO_GRAB = 0.7,
            SERVO_RELEASE = 0,
            FULL_SPEED=1,
            ZERO_SPEED=0,
            SLOW_DOWN = 1.7;
    //boolean
    boolean   gpad1x, gpad1y, gpad2a,gpad2b,gpad2rightBumper,gpad2leftBumper;

    private void printStatus() {
        telemetry.addData("rightservo current position", rightFoundation.getPosition());
        telemetry.addData("leftservo current position", leftFoundation.getPosition());
        //telemetry.addData("armservo current position", grabber.getPosition());
        //telemetry.addData("y stick value:", gamepad1.right_stick_y);
        //telemetry.addData("x stick value:", gamepad1.right_stick_y);
       // telemetry.addData("y stick value:", gamepad1.right_stick_y);
        telemetry.addData("x", gpad1x);
        telemetry.update();


    }


    @Override
    public void runOpMode() throws InterruptedException {
        //Init hardware
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        arm = hardwareMap.get(DcMotor.class, "arm");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo = hardwareMap.get(CRServo.class, "armServo");


         //buttons g1xButton, g1yButton, g2aButton,g2bButton,g2rightBump,g2leftBump;
         gpad1x=gamepad1.x;
         gpad1y=gamepad1.y;
         gpad2a=gamepad2.a;
         gpad2b=gamepad2.b;
         gpad2rightBumper=gamepad2.right_bumper;
         gpad2leftBumper=gamepad2.left_bumper;

        // rightMotor is upside-down
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //zero power behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // wait for play
        waitForStart();
        while (opModeIsActive()) {

            if (gpad1x) {
                grabFoundation();


               }
            if (gpad1y) {
                releaseFoundation();
                                            
            }
            if (gpad2a) {
                servoRelease();
            }
            if (gpad2b) {
                servoGrab();
            }

            arm();//Arm function

            drive();//Drive function
            telemetry.addData("rightservo current position", rightFoundation.getPosition());
            telemetry.addData("leftservo current position", leftFoundation.getPosition());   printStatus();
            telemetry.addData("x", gpad1x);        
        }   telemetry.update();                    

    }
    // Foundation code
    private void releaseFoundation(){
        rightFoundation.setPosition(RIGHT_FOUNDATION_UP);
        leftFoundation.setPosition(LEFT_FOUNDATION_UP);
    }
    private void grabFoundation(){
        rightFoundation.setPosition(RIGHT_FOUNDATION_DOWN);
        leftFoundation.setPosition(LEFT_FOUNDATION_DOWN);
    }

    //Grabber code
    private void servoGrab(){
        grabber.setPosition(SERVO_GRAB);
    }
    private void servoRelease(){
        grabber.setPosition(SERVO_RELEASE);
    }

    //Arm code
    private void arm(){
        if (gpad2rightBumper){
            armServo.setPower(FULL_SPEED);

        } else if (gpad2leftBumper) {
            armServo.setPower(-FULL_SPEED);
        } else {

            armServo.setPower(ZERO_SPEED);
        }

        arm.setPower(gamepad2.left_trigger);
        arm.setPower(-gamepad2.right_trigger);
    }

    //Drive Code
    private void drive(){
        double leftPower = -gamepad1.left_stick_y;
        left.setPower(leftPower /SLOW_DOWN);
        double rightPower = -gamepad1.right_stick_y;
        right.setPower(rightPower /SLOW_DOWN);
    }




}

