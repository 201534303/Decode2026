package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Shooter {

    protected Telemetry telemetry;
    protected DcMotorEx shooterR;
    protected DcMotorEx shooterL;
    protected double rotSpeed;
    protected ElapsedTime runtime;
    protected double lastTime;
    protected double lastTicks;
    

    public Shooter(HardwareMap h, Telemetry t, ElapsedTime r){
        shooterR = h.get(DcMotorEx.class, "shooterR");
        shooterL = h.get(DcMotorEx.class, "shooterL");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = t;
        runtime = r;
    }

    protected void setPower(double p){
        shooterR.setPower(p);
        shooterL.setPower(p);
    }
    
    protected void setSpeed(double s){
        
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

    //TODO add PID functions
}
