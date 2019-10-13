package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    public DcMotor motor;

    public Hardware(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivePower(double power) {
        motor.setPower(power);
    }

}
