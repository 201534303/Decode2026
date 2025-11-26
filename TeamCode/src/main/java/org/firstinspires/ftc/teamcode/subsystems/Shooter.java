package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Shooter {

    protected Telemetry telemetry;
    protected MotorEx shooterR;
    protected MotorEx shooterL;
    protected double rotSpeed;
    protected ElapsedTime runtime;
    protected double lastTime;
    protected double lastTicks;
    protected PIDFController PIDF;
    public double power = 0;
    protected double idealSpeed;
    double last_error = 0;
    double integral = 0;

    public Shooter(HardwareMap h, Telemetry t, ElapsedTime r){
        shooterR = h.get(MotorEx.class, "shooterR");
        shooterL = h.get(MotorEx.class, "shooterL");


        shooterL.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterR.setRunMode(Motor.RunMode.VelocityControl);
        shooterL.setRunMode(Motor.RunMode.VelocityControl);

        telemetry = t;
        runtime = r;
        //PIDF = new PIDFController(0.001,0,0, .55);
    }

    public double getMotorVel(){
        return shooterR.getVelocity();
    }

    protected void setVel(double flywheelV){
        shooterR.setVelocity(flywheelV);
        shooterL.setVelocity(flywheelV);
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

    public void flywheelSpin(double targetVelo, double currentVelo){
        double speed = PIDF(targetVelo-currentVelo, targetVelo, 1,0,0,0.3);
        shooterR.setVelocity(-speed);
        shooterL.setVelocity(speed);
    }

    public double PIDF(double error, double setpoint, double kp, double ki, double kd, double kF) {

        integral += error;
        double derivative = error - last_error;

        double proportional = error * kp;
        double integralTerm = integral * ki;
        double derivativeTerm = derivative * kd;

        // Feedforward = kF * setpoint
        double feedforward = kF * setpoint;

        double correction = proportional + integralTerm + derivativeTerm + feedforward;

        last_error = error;

        return correction;
    }


    //TODO add PID functions
}
