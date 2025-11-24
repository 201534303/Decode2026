package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Drivetrain {

    protected DcMotor frontLeft, frontRight, backLeft, backRight;

    protected Telemetry telemetry;

    protected IMU imu;
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public Drivetrain(HardwareMap hardwareMap, Telemetry t){
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackMotor");
        backRight = hardwareMap.get(DcMotor.class, "rightBackMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = t;

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
    }

    protected void driveRobot(double drive, double strafe, double turn){
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeft.setPower((drive+strafe+turn)*-1);
        frontRight.setPower((drive-strafe-turn)*-1);
        backLeft.setPower((drive-strafe+turn)*-1);
        backRight.setPower((drive+strafe-turn)*-1);
    }

}
