package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.OUT;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class Intake {

    protected ElapsedTime timer;
    protected double timeOld;
    protected double direction = 1;

    protected DcMotorEx intake;
    protected DcMotorEx transfer;
    DigitalChannel right,left, front1, front2, distFront;
    protected double iSpeed = 0;
    protected double tSpeed = 0;
    protected Telemetry telemetry;
    protected Intake.intakeState intakeState = OFF;
    double last_error = 0;
    double integral = 0;

    public Intake(HardwareMap hardwareMap, Telemetry t, ElapsedTime e){

        timer = e;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        right = hardwareMap.get(DigitalChannel.class, "distRight");
        left = hardwareMap.get(DigitalChannel.class, "distLeft");
        front1 = hardwareMap.get(DigitalChannel.class, "colorRight");
        front2 = hardwareMap.get(DigitalChannel.class, "colorLeft");
        distFront = hardwareMap.get(DigitalChannel.class, "distFront");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        //transfer.setRunMode(MotorEx.RunMode.VelocityControl);

        telemetry = t;
    }

    public void setIntPower(double power){
        iSpeed = power;
    }
    public void intakeIn(){
        intakeState = IN;
    }


    public void intakeOut(){
        intakeState = OUT;
    }

    public void intakeOff(){
        intakeState = OFF;
    }



    public double getTransferVel(){
        return transfer.getVelocity();
    }
    public void setTransferPower(double power){
        transfer.setPower(power);
    }

    public void setTransferVelPID(double vel, double currentVelo, double stickInput, double tuner){
        telemetry.addData("We are settin transfer vel", vel);
        //transfer.setRunMode(MotorEx.RunMode.VelocityControl);
        double speed = PIDF(vel-currentVelo, vel, 0.15,0,0,1.08);
        //1.15
        transfer.setVelocity(speed);
    }


    //Not used
    public void setIntakePower(double power){
        telemetry.addData("We are setting intake", power);
        intake.setPower(power);
        //transfer.setPower(iSpeed);
    }


    public void intakeMachine(){
        switch (intakeState){
            case IN:
                intake.setPower(iSpeed);
                setTransferPower(-iSpeed);
                break;
            case OUT:
                intake.setPower(-iSpeed);
                break;
            case OFF:
                intake.setPower(0);
                setTransferPower(0);
                break;
        }
    }

    public enum intakeState{
        IN, OUT, OFF
    }

    public boolean haveBall(){
        if (left.getState() || right.getState()){
            if (distFront.getState() || front1.getState()){
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
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
