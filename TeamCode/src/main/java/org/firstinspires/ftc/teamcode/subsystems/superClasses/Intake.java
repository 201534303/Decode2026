package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake.intakeState.OUT;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    protected DcMotorEx intake, transfer;
    DigitalChannel right,left;
    Servo taillight;
    protected double iSpeed = 0;
    protected double tSpeed = 0;
    protected Telemetry telemetry;
    protected Intake.intakeState intakeState = OFF;

    public Intake(HardwareMap hardwareMap, Telemetry t){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        right = hardwareMap.get(DigitalChannel.class, "distRight");
        left = hardwareMap.get(DigitalChannel.class, "distLeft");
        taillight = hardwareMap.get(Servo.class, "taillight");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void setTransferPower(double power){
        telemetry.addData("We are settin transfer", power);
        tSpeed = power * 0.7;
        transfer.setPower(tSpeed);
    }

    public void setIntakePower(double power){
        telemetry.addData("We are setting intake", power);
        iSpeed = power;
        transfer.setPower(iSpeed);
    }


    public void intakeMachine(){
        switch (intakeState){
            case IN:
                intake.setPower(iSpeed);
                break;
            case OUT:
                intake.setPower(-iSpeed);
                break;
            case OFF:
                intake.setPower(0);
                break;
        }
    }

    public enum intakeState{
        IN, OUT, OFF
    }

    public boolean haveBall(){
        return right.getState() || left.getState() == true;
    }

    public void noBallLight(){
        taillight.setPosition(0.3);
    }
    public void yesBallLight(){
        taillight.setPosition(0.5);
    }

}
