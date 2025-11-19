package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTele extends Intake{
    protected Gamepad gamepad1, gamepad2;

    public IntakeTele(HardwareMap hardwareMap, Gamepad g1, Gamepad g2, Telemetry t) {
        super(hardwareMap, t);
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void update(){
        if((gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) || gamepad1.right_trigger < 0.1 || gamepad1.left_trigger < 0.1){
            intakeOff();
        }
        if(gamepad1.right_trigger > .1){
            setPower(gamepad1.right_trigger);
            intakeIn();
        }
        if(gamepad1.left_trigger > .1){
            setPower(gamepad1.left_trigger);
            intakeOut();
        }
        intakeMachine();
    }
}
