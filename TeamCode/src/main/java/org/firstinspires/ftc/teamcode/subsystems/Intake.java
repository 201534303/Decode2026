package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Intake {
    protected DcMotorEx intake, transfer;
    protected double iSpeed = 0;
    protected double tSpeed = 0;
    protected Telemetry telemetry;
    protected Intake.intakeState intakeState = OFF;

    public Intake(HardwareMap hardwareMap, Telemetry t){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = t;
    }

    protected void setIntPower(double power){
        iSpeed = power;
    }
    protected void intakeIn(){
        intakeState = IN;
    }

    protected void intakeOut(){
        intakeState = OUT;
    }

    protected void intakeOff(){
        intakeState = OFF;
    }

    protected void setTransferPower(double power){
        tSpeed = power;
        transfer.setPower(tSpeed);
    }

    protected void intakeMachine(){
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

}
