package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    //motors
    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected Servo stopperRight, stopperLeft;

    //telemetry
    protected Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry t) {

        //motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackMotor");
        backRight = hardwareMap.get(DcMotor.class, "rightBackMotor");

        //reverse directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //break
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry
        telemetry = t;

        //servo
        stopperRight = hardwareMap.get(Servo.class, "stopperRight");
        stopperLeft = hardwareMap.get(Servo.class, "stopperLeft");

    }

    public void driveRobot(double drive, double strafe, double turn) {
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeft.setPower((drive + strafe + turn) * -1);
        frontRight.setPower((drive - strafe - turn) * -1);
        backLeft.setPower((drive - strafe + turn) * -1);
        backRight.setPower((drive + strafe - turn) * -1);
    }

    public void driveRobotHeading(double drive, double idealHeading, double heading) {
        double error = 2*angleDiffRad(heading, idealHeading);
        double den = Math.abs(error) + Math.abs(drive);
        frontLeft.setPower((drive - error)/den);
        frontRight.setPower((drive + error)/den);
        backLeft.setPower((drive - error)/den);
        backRight.setPower((drive + error)/den);
    }

    public static double angleDiffRad(double fromRad, double toRad) {
        double diff = (toRad - fromRad) % (2.0 * Math.PI);
        if (diff > Math.PI) diff -= 2.0 * Math.PI;
        if (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff; // range: [-PI, PI]
    }


    public void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void setStopper1Pos(double pos) {
        stopperRight.setPosition(pos);
    }

    public void lock() {
        stopperRight.setPosition(0.15);
        stopperLeft.setPosition(0);


    }
    public void unlock() {
        stopperRight.setPosition(0.5);
        stopperLeft.setPosition(0.5);
    }
}