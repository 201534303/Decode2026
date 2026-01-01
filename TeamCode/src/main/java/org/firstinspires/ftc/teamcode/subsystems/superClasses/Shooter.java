package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    //telemetry
    protected Telemetry telemetry;

    //motors
    MotorEx shooterR;
    MotorEx shooterL;

    //runtime
    protected ElapsedTime runtime;

    public double power = 0;
    protected double idealSpeed;
    double last_error = 0;
    double integral = 0;
    protected Servo hood;
    Servo right, left;
    AnalogInput leftEnc;

    public Shooter(HardwareMap hardwareMap, Telemetry t, ElapsedTime r){
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

        //break
        shooterL.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //VelocityMode
        shooterR.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterL.setRunMode(MotorEx.RunMode.VelocityControl);

        telemetry = t;
        runtime = r;
    }

    public double getMotorVel(){
        //in ticks/sec - gobilda bare is 28 tps
        return shooterL.getVelocity();
    }

    public double getMotorRPM(){
        //gets rotations per sec then converts it to rpm
        return (shooterL.getVelocity()/28)*60;
    }

    public double RPMToVel(double RPM){
        return (RPM/60)*28;
    }

    public void setVel(double flywheelV){
        shooterR.setVelocity(flywheelV);
        shooterL.setVelocity(flywheelV);
    }

    protected void setRPM(double flywheelRPM){
        shooterR.setVelocity(RPMToVel(flywheelRPM));
        shooterL.setVelocity(RPMToVel(flywheelRPM));
    }

    public void flywheelSpin(double targetVelo, double currentVelo, double kf){//kf is a tester varible
        double speed = PIDF(targetVelo-currentVelo, targetVelo, 12,0,0.1,0.59);
        shooterR.setVelocity(speed);
        shooterL.setVelocity(speed);
    }

    public void rotateTurret(double theta){
        theta = normalizeDeg(theta);

        //hard stops
        if (theta > 63){
            theta = 63;
        }
        if (theta < -63){
            theta = -63;
        }

        //setting it
        theta = 0.48 /*center*/ + theta * (1/(71.35*2));
        telemetry.addData("image!", "71.35*2!");
        right.setPosition(theta);
        left.setPosition(theta);
    }

    public void seTurretRaw(double theta){
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
        if(theta < 0.05){
            theta = 0.05;
        }
        theta = 1-theta;
        hood.setPosition(theta);
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

    public double PIDF(double error, double setpoint, double kp, double ki, double kd, double kF, double x, double y) {

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
