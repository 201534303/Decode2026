package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.IN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.intakeState.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intake;
    public Intake.intakeState intakeState = OFF;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void intakeIn(){
        intake.setPower(-1);
    }

    public void intakeOut(){
        intake.setPower(1);
    }

    public void intakeOff(){
        intake.setPower(0);
    }

    public void intakeMachine(double in, double out){
        switch (intakeState){
            case IN:
                if (in > 0.9) {
                    intakeIn();
                } else if (out > 0.9) {
                    intakeState = OUT;
                } else {
                    intakeState = OFF;
                }
                break;
            case OUT:
                if (out > 0.9) {
                    intakeOut();
                } else if (in > 0.9) {
                    intakeState = IN;
                } else {
                    intakeState = OFF;
                }
                intakeOut();
                break;
            case OFF:
                if (in > 0.9) {
                    intakeIn();
                } else if (out > 0.9) {
                    intakeState = OUT;
                } else {
                    intakeOff();
                }
                intakeOff();
                break;
        }

    }

    public enum intakeState{
        IN, OUT, OFF
    }

}
