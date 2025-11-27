package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Shooter {

    protected Telemetry telemetry;
    MotorEx shooterR;
    MotorEx shooterL;
    protected double rotSpeed;
    protected ElapsedTime runtime;
    protected double lastTime;
    protected double lastTicks;
    protected PIDFController PIDF;
    public double power = 0;
    protected double idealSpeed;
    double last_error = 0;
    double integral = 0;

    public Shooter(HardwareMap hardwareMap, Telemetry t, ElapsedTime r){
        //shooterR = hardwareMap.get(MotorEx.class, "shooterR");
        //shooterL = hardwareMap.get(MotorEx.class, "shooterL");
        shooterR = new MotorEx(hardwareMap, "shooterR", MotorEx.GoBILDA.BARE);
        shooterL = new MotorEx(hardwareMap, "shooterL", MotorEx.GoBILDA.BARE);

        shooterL.setInverted(true);

        shooterL.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterR.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterL.setRunMode(MotorEx.RunMode.VelocityControl);

        telemetry = t;
        runtime = r;
        //PIDF = new PIDFController(0.001,0,0, .55);
    }

    public double getMotorVel(){
        //in ticks/sec - gobilda bare is 28 tps
        return shooterL.getVelocity();
    }

    public double getMotorRPM(){
        //gets rotations per sec then converts it to rpm
        return (shooterL.getVelocity()/28)*60;
    }

    public double velToRPM(double vel){
        return (vel/28)*60;
    }

    public double RPMToVel(double RPM){
        return (RPM/60)*28;
    }

    protected void setVel(double flywheelV){
        shooterR.setVelocity(flywheelV);
        shooterL.setVelocity(flywheelV);
    }

    protected void setRPM(double flywheelRPM){
        shooterR.setVelocity(RPMToVel(flywheelRPM));
        shooterL.setVelocity(RPMToVel(flywheelRPM));
    }
    
    protected void setSpeed(double s){
        idealSpeed = s;
    }

    /*
    protected void updateRoot(){
        power = PIDF.calculate(rotSpeed(), idealSpeed);
        setVel(power);
        telemetry.addData("power", power);
    }

    protected double rotSpeed(){
        double timeChatnge =  runtime.time() - lastTime;
        double tickChange = shooterL.getCurrentPosition() -lastTicks;
        double rotSpeed = tickChange/(28.0*timeChatnge);
        telemetry.addData("RPM", rotSpeed);
        lastTime = runtime.time();
        lastTicks = shooterL.getCurrentPosition();
        return rotSpeed;
    }

     */

    public void flywheelSpin(double targetVelo, double currentVelo, double kf){//kf is a tester varible
        double speed = PIDF(targetVelo-currentVelo, targetVelo, 9.5,0,0.1,0.59);
        shooterR.setVelocity(speed);
        shooterL.setVelocity(speed);
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
