package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected IMU imu;
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    protected Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry t){
        frontLeft = hardwareMap.get(DcMotor.class, "drivefl");
        frontRight = hardwareMap.get(DcMotor.class, "drivefr");
        backLeft = hardwareMap.get(DcMotor.class, "drivebl");
        backRight = hardwareMap.get(DcMotor.class, "drivebr");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = t;

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    protected void driveRobot(double drive, double strafe, double turn){
        frontLeft.setPower((drive+strafe+turn)*-1);
        frontRight.setPower((drive-strafe-turn)*-1);
        backLeft.setPower((drive-strafe+turn)*-1);
        backRight.setPower((drive+strafe-turn)*-1);
    }


}
