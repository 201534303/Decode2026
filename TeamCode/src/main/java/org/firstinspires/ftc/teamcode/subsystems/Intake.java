package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Intake {
    protected DcMotor intake;
    protected double speed = 0;
    protected Telemetry telemetry;
    protected Intake.intakeState intakeState = OFF;

    public Intake(HardwareMap hardwareMap, Telemetry t){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = t;
    }

    protected void setPower(double power){
        speed = power;
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

    protected void intakeMachine(){
        switch (intakeState){
            case IN:
                intake.setPower(speed);
                break;
            case OUT:
                intake.setPower(-speed);
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
