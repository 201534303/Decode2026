package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class Shooter {

    //telemetry
    protected Telemetry telemetry;

    //motors
    MotorEx shooterR;
    MotorEx shooterL;

    //runtime
    protected ElapsedTime runtime;

    double last_error = 0;
    double integral = 0;
    protected Servo hood;
    Servo right, left;
    AnalogInput leftEnc;
    public double thetaT;
    double speed;

    private double tv;
    private double cv;
    private double rv;



    public Shooter(HardwareMap hardwareMap, Telemetry t, ElapsedTime r) {
        //init servos and motors
        shooterR = new MotorEx(hardwareMap, "shooterR", MotorEx.GoBILDA.BARE);
        shooterL = new MotorEx(hardwareMap, "shooterL", MotorEx.GoBILDA.BARE);
        right = hardwareMap.get(Servo.class, "turret_right");
        left = hardwareMap.get(Servo.class, "turret_left");
        hood = hardwareMap.get(Servo.class, "hood");

        //notUsed
        leftEnc = hardwareMap.get(AnalogInput.class, "turrentencoder");

        //invertMotor
        shooterR.setInverted(true);



        telemetry = t;
        runtime = r;
    }

    public double getMotorVel() {
        return (shooterL.getVelocity());

    }

    public double getMotorRPM() {
        //gets rotations per sec then converts it to rpm
        return (shooterL.getVelocity() / 28) * 60;
    }

    public double RPMToVel(double RPM) {
        return (RPM / 60) * 28;
    }

    public void setVel(double flywheelV) {
        shooterR.setVelocity(flywheelV);
        shooterL.setVelocity(flywheelV);
    }

    protected void setRPM(double flywheelRPM) {
        shooterR.setVelocity(RPMToVel(flywheelRPM));
        shooterL.setVelocity(RPMToVel(flywheelRPM));
    }

    public void flywheelSpinBangBang(double targetVelo, double currentVelo, double kf) {//kf is a tester varible
        if (targetVelo - currentVelo <= 0) {
            speed = 0;
        } else {
            speed = 1;
        }
        shooterL.set(speed);
        shooterR.set(speed);
        telemetry.addData("target velocity", Math.round(targetVelo * 10000) / 10000.0);
        telemetry.addData("current velocity", Math.round(currentVelo * 10000) / 10000.0);
        telemetry.addData("velo difference", Math.round((currentVelo-targetVelo) * 10000) / 10000.0);

    }

    public void flywheelSpinDynamicLoop(){
        flywheelSpinDynamic(tv, cv, rv);
    }
    public void flywheelSpinDynamic(double targetVelo, double currentVelo, double robotVel) {

        if (currentVelo > targetVelo + 60){
            shooterR.setRunMode(MotorEx.RunMode.VelocityControl);
            shooterL.setRunMode(MotorEx.RunMode.VelocityControl);
            double speedPID = PIDF(targetVelo-currentVelo, targetVelo, 12,0,0.1,0.59);
            shooterR.setVelocity(speedPID);
            shooterL.setVelocity(speedPID);
        } else {
            shooterR.setRunMode(MotorEx.RunMode.RawPower);
            shooterL.setRunMode(MotorEx.RunMode.RawPower);
            if (targetVelo - currentVelo <= 0) {
                speed = 0;
            } else {
                speed = 1;
            }
            shooterL.set(speed);
            shooterR.set(speed);
        }

        telemetry.addData("target velocity", Math.round(targetVelo * 10000) / 10000.0);
        telemetry.addData("current velocity", Math.round(currentVelo * 10000) / 10000.0);

        telemetry.addData("velo difference", Math.round((currentVelo-targetVelo) * 10000) / 10000.0);
        tv = targetVelo;
        cv = currentVelo;
        rv = robotVel;
    }

    public double getTargetVelocity(){
        return tv;
    }

    public void rotateTurret(double theta) {
        telemetry.addData("setTurret", Math.round(theta * 100) / 100.0);
        theta = normalizeDeg(theta);

        thetaT = theta;

        //hard stops
        if (theta > 73) {//72
            theta = 73;
        }
        if (theta < -73) {
            theta = -73;
        }

        //setting it
        theta = 0.50 /*center*/ + theta * (1.74 / (360.0) * 1.40);
        right.setPosition(theta);
        left.setPosition(theta);
        telemetry.addData("rawTurret", theta);
    }

    public void rotateTurretZeroTest(double theta) {
        right.setPosition(theta);
        left.setPosition(theta);
    }

    public void rotateTurretConversionTest(double theta, double mul) {
        telemetry.addData("turret", Math.round(theta * 100) / 100.0);
        theta = normalizeDeg(theta);

        //thetaT = theta;

        //hard stops
        if (theta > 75) {
            theta = 75;
        }
        if (theta < -75) {
            theta = -75;
        }

        //setting it
        theta = 0.5025 /*center*/ + theta * (1.74 / (360.0) * mul);
        thetaT = theta;
        right.setPosition(theta);
        left.setPosition(theta);
    }

    public static double normalizeDeg(double angleDeg) {
        angleDeg = angleDeg % 360.0;
        if (angleDeg > 180.0) {
            angleDeg -= 360.0;
        } else if (angleDeg <= -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    public void setHood(double theta) {
        theta = 1 - theta;
        hood.setPosition(theta);
        telemetry.addData("raw hood", Math.round(theta * 100) / 100.0);
    }

    public double servoPos() {
        return hood.getPosition();
    }


    public double PIDF(double error, double setpoint, double kp, double ki, double kd, double kF) {

        integral += error;
        double derivative = error - last_error;

        double proportional = error * kp;
        double integralTerm = integral * ki;
        double derivativeTerm = derivative * kd;

        // Feedforward = kF * setpoint; important for scaling feedforward
        double feedforward = kF * setpoint;

        double correction = proportional + integralTerm + derivativeTerm + feedforward;

        last_error = error;

        return correction;
    }
}
