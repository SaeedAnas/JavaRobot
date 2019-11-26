package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp (name="Macanum-Teleop1.java", group="teloop")
public class Teleop extends LinearOpMode {
    //Hardware
    private DcMotor rearLeft,rearRight,frontLeft,frontRight, arm, arm2;
    private Servo rightFoundation, leftFoundation, grabber, sideServo;
    private CRServo armServo;
    //Constants
    private static final double
            LEFT_FOUNDATION_DOWN = 0.4,
            LEFT_FOUNDATION_UP = 0.9,
            RIGHT_FOUNDATION_DOWN = 0.6,
            RIGHT_FOUNDATION_UP = 0.1,
            SERVO_GRAB = 0.3,
            SERVO_RELEASE = 0,
            FULL_SPEED=1,
            ZERO_SPEED=0,
            SLOW_DOWN1 = 1.3,
            SLOW_DOWN2 = 1.5;
    private int driver = 0;
    //boolean
    boolean   gpad1x, gpad1y, gpad2a,gpad2b,gpad2rightBumper,gpad2leftBumper;

//



    private void printStatus() {

        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //initImu();
        //Init hardware
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight= hardwareMap.get(DcMotor.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontLeft");
        arm = hardwareMap.get(DcMotor.class, "armLeft");
        arm2 =hardwareMap.get(DcMotor.class,"armRight");
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
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        //zero power behavior
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for play
        waitForStart();
        while (opModeIsActive()) {
            //printStatus();
            if (gamepad1.x) {

                rightFoundation.setPosition(0.6);
                leftFoundation.setPosition(0.4);

            }
            if (gamepad1.y) {
                rightFoundation.setPosition(0.1);
                leftFoundation.setPosition(0.9);

            }
            if (gamepad2.a) {
                grabber.setPosition(0.7);

            }
            if (gamepad2.b) {
                grabber.setPosition(0);
            }
            if (gamepad2.x) {
                armServo.setPower(0.7);

            }
            else if (gamepad2.y) {
                armServo.setPower(-0.7);
            } else {

                armServo.setPower(0);
            }
            if (gamepad2.right_trigger>0.5){
                arm.setPower(-1);
                arm2.setPower(-1);

            } else if (gamepad2.left_trigger>0.5) {
                arm.setPower(1);
                arm2.setPower(1);
            } else {

                arm.setPower(0);
                arm2.setPower(0);
            }
            //arm();//Arm function

            drive();//Drive function
//            telemetry.addData("grabber",armServo.getDirection());
//            telemetry.addData("leftbumper",gamepad2.left_bumper);
//            telemetry.addData("gamepad2x",gamepad2.x);
//            telemetry.addData("left trigger",gamepad2.left_trigger);
//            telemetry.addData("right trigger",gamepad2.right_trigger);
//            telemetry.update();
        }

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
//    private void arm(){
//        if (gpad2rightBumper){
//            //armServo.setDirection(1);
//
//        } else if (gpad2leftBumper) {
//            //armServo.setDirection(-1);
//        } else {
//
//            armServo.setPower(ZERO_SPEED);
//        }
//
//
//
//
//
//        arm.setPower(gamepad2.left_trigger);
//        arm.setPower(-gamepad2.right_trigger);
//    }

    //Drive Code
    private void drive(){
//        double leftPower1 = gamepad1.right_stick_y;
//        double rightPower1 = gamepad1.left_stick_y;
//        double leftPower2 = gamepad2.right_stick_y;
//        double rightPower2 = gamepad2.left_stick_y;

//        if(driver == 0) {
//            if(leftPower1 != 0 || rightPower1 != 0) {
//                right.setPower(rightPower1/SLOW_DOWN1);
//                left.setPower(leftPower1/SLOW_DOWN1);
//                driver = 1;
//            } else if (leftPower2 != 0 || rightPower2 != 0) {
//                right.setPower(rightPower2/SLOW_DOWN2);
//                left.setPower(leftPower2/ SLOW_DOWN2);
//                driver = 2;
//            }
//        } else if (driver == 1) {
//            if(leftPower1 != 0 || rightPower1 != 0) {
//                right.setPower(rightPower1/SLOW_DOWN1);
//                left.setPower(leftPower1/SLOW_DOWN1);
//            }
//            else {
//                driver = 0;
//                left.setPower(0);
//                right.setPower(0);
//            }
//        } else if (driver == 2){
//            if (leftPower2 != 0 || rightPower2 != 0) {
//                right.setPower(rightPower2/SLOW_DOWN2);
//                left.setPower(leftPower2/ SLOW_DOWN2);
//            } else {
//                driver = 0;
//                left.setPower(0);
//                right.setPower(0);
//            }
//        }
        // forward and backward motion     // ask Peter about this
        if (gamepad1.left_stick_y!=0 && gamepad1.left_stick_x<0.3 || gamepad1.left_stick_x >0.3) {
            rearRight.setPower(gamepad1.left_stick_y);
            rearLeft.setPower(gamepad1.left_stick_y);
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.left_stick_y);
        }else {
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //sliding motion left
        if (gamepad1.left_stick_x!=0 && gamepad1.left_stick_y<0.3 || gamepad1.left_stick_y >0.3) {
            rearLeft.setPower()(-gamepad1.left_stick_x); // negative negative cancels out  confirm and test
            rearRight.setPower(gamepad1.left_stick_x);
            frontLeft.setPower(gamepad1.left_stick_x);
            frontRight.setPower(-gamepad1.left_stick_x);
        }else {
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
        // sliding motion right  & Copy of Sliding Motion Left       TEST THIS
        if (gamepad1.left_stick_x!=0 && gamepad1.left_stick_y<0.3 || gamepad1.left_stick_y >0.3) {
            rearLeft.setPower(-gamepad1.left_stick_x);
            rearRight.setPower(gamepad1.left_stick_x);
            frontLeft.setPower(gamepad1.left_stick_x);
            frontRight.setPower(-gamepad1.left_stick_x);
        }else {
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Rotate Left
        if (gamepad1.left_trigger!=0) {
            frontLeft.setPower(gamepad1.left_trigger);
            rearLeft.setPower(gamepad1.left_trigger);
            frontRight.setPower(-gamepad1.left_trigger);
            rearRight.setPower(-gamepad1.left_trigger);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Rotate Right
        if (gamepad1.right_trigger!=0) {
            frontLeft.setPower(-gamepad1.right_trigger);
            rearLeft.setPower(-gamepad1.right_trigger);
            frontRight.setPower(gamepad1.right_trigger);
            rearRight.setPower(gamepad1.right_trigger);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Strafe Top-Left
        if (gamepad1.right_stick_y!=0 && gamepad1.right_stick_x<0.3 || gamepad1.right_stick_x>0.3) {
            frontLeft.setPower(0);
            rearLeft.setPower(gamepad1.right_stick_y);
            frontRight.setPower(gamepad1.right_stick_y);
            rearRight.setPower(0);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Strafe Bottom-Right & Copy of Top-Left
        if (gamepad1.right_stick_y!=0 && gamepad1.right_stick_x<0.3 || gamepad1.right_stick_x>0.3) {
            frontLeft.setPower(0);
            rearLeft.setPower(gamepad1.right_stick_y);
            frontRight.setPower(gamepad1.right_stick_y);
            rearRight.setPower(0);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Strafe Top-Right
        if (gamepad1.right_stick_x!=0 && gamepad1.right_stick_y<0.3 || gamepad1.right_stick_y>0.3) {
            frontLeft.setPower(-gamepad1.right_stick_x);
            rearLeft.setPower(0);
            frontRight.setPower(0);
            rearRight.setPower(-gamepad1.right_stick_x);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //Strafe Bottom-Left & Copy of Strafe Top-Right
        if (gamepad1.right_stick_x!=0 && gamepad1.right_stick_y<0.3 || gamepad1.right_stick_y>0.3) {
            frontLeft.setPower(-gamepad1.right_stick_x);
            rearLeft.setPower(0);
            frontRight.setPower(0);
            rearRight.setPower(-gamepad1.right_stick_x);
        }else{
            rearRight.setPower(0);
            rearLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }







    }
}
