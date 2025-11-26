package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DrivetrainTele extends Drivetrain{

    private Gamepad gamepad1, gamepad2;


    public DrivetrainTele(HardwareMap hardwareMap, Gamepad g1, Gamepad g2, Telemetry t) {
        super(hardwareMap, t);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void feildCentricDrive(){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = gamepad1.left_stick_x;

        double brake = gamepad1.right_trigger;
        double superBrake = gamepad1.left_trigger;

        if (gamepad1.options){
            imu.resetYaw();
        }

        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotedX = xMove * Math.cos(botHeading) - yMove * Math.sin(botHeading);
        double rotedY = xMove * Math.sin(botHeading) + yMove * Math.cos(botHeading);

        telemetry.addData("rotedY", rotedY);
        telemetry.addData("rotedX", rotedX);

        rotedX = rotedX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        //double denominator = Math.max(Math.abs(rotedY) + Math.abs(rotedX) + Math.abs(rot), 1);
        double frontLeftPower = (rotedY + rotedX + rot); /// denominator;
        double backLeftPower = (rotedY - rotedX + rot); /// denominator;
        double frontRightPower = (rotedY - rotedX - rot); /// denominator;
        double backRightPower = (rotedY + rotedX - rot); /// denominator;
        if (brake > 0.9){
            frontLeft.setPower(frontLeftPower * 0.6);
            backLeft.setPower(backLeftPower* 0.6);
            frontRight.setPower(frontRightPower* 0.6);
            backRight.setPower(backRightPower* 0.6);
        } else if (superBrake > 0.9) {
            frontLeft.setPower(frontLeftPower * 0.35);
            backLeft.setPower(backLeftPower* 0.35);
            frontRight.setPower(frontRightPower* 0.35);
            backRight.setPower(backRightPower* 0.35);
        } else{
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
        telemetry.addData("xHeading", rotedX);
        telemetry.addData("xHeading", rotedX);
    }

    public void printData(){
        telemetry.addData("Imu",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

}
