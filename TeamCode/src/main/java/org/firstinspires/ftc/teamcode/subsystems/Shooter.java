package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    final double YOFFSET = 1.0;
    final double XOFFSET = 1.0;
    protected Servo hood;
    CRServo right, left;
    AnalogInput leftEnc;

    public Shooter(HardwareMap hardwareMap, Telemetry t, ElapsedTime r){
        //shooterR = hardwareMap.get(MotorEx.class, "shooterR");
        //shooterL = hardwareMap.get(MotorEx.class, "shooterL");
        shooterR = new MotorEx(hardwareMap, "shooterR", MotorEx.GoBILDA.BARE);
        shooterL = new MotorEx(hardwareMap, "shooterL", MotorEx.GoBILDA.BARE);
        right = hardwareMap.get(CRServo.class, "turret_right");
        left = hardwareMap.get(CRServo.class, "turret_left");
        hood = hardwareMap.get(Servo.class, "hood");
        leftEnc = hardwareMap.get(AnalogInput.class, "turrentencoder");


        shooterR.setInverted(true);

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
        double speed = PIDF(targetVelo-currentVelo, targetVelo, 12,0,0.1,0.59);
        shooterR.setVelocity(speed);
        shooterL.setVelocity(speed);
    }

    public void setTurretpos(double pos){
        //1.6 is zero offset
        double logicalPos = leftEnc.getVoltage() - 1.6;
        if (logicalPos > 1.3){
            logicalPos = 1.3;
        }
        if (logicalPos < -1.3){
            logicalPos = -1.3;
        }
        double power = PIDF(pos- logicalPos, pos, 0.53,0.0003,0.35,0);
        //double power = PIDF(pos- logicalPos, pos, 0.5,0,0,0);

        left.setPower(power);
        right.setPower(power);
    }

    public void setTurretPower(double power){
        left.setPower(power);
        right.setPower(power);
    }



    public double getTTPos(){
        //1.6 is 0 offset
        return leftEnc.getVoltage() - 1.6;
    }
    /*
    public void rotateTurret(double theta){
        //right = -0.4695
        //left = 0.4861
        theta = theta/360 * 2;
        if(theta < -0.4845){
            theta = -0.4845;
        }
        if(theta > 0.4811){
            theta = 0.4811;
        }
        theta = theta + 0.5 + + 0.0072;

        theta = 1-theta;
        right.setPosition(theta);
        left.setPosition(theta);
    }

     */

    public void hoodPitch(double theta) {
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
}
