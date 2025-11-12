package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    public Drivetrain(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class, "drivefl");
        frontRight = hardwareMap.get(DcMotor.class, "drivefr");
        backLeft = hardwareMap.get(DcMotor.class, "drivebl");
        backRight = hardwareMap.get(DcMotor.class, "drivebr");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveRobot(double drive, double strafe, double turn){

        frontLeft.setPower((drive+strafe+turn)*-1);
        frontRight.setPower((drive-strafe-turn)*-1);
        backLeft.setPower((drive-strafe+turn)*-1);
        backRight.setPower((drive+strafe-turn)*-1);
    }

}
