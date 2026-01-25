package org.firstinspires.ftc.teamcode.subsystems.OldTele;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;

public class IntakeTele extends Intake {
    protected Gamepad gamepad1, gamepad2;

    public IntakeTele(HardwareMap hardwareMap, Gamepad g1, Gamepad g2, Telemetry t, ElapsedTime e) {
        super(hardwareMap, t, e);
        gamepad1 = g1;
        gamepad2 = g2;
    }

    
    public void update(){
        setIntPower(-gamepad2.right_stick_y + 0.1);
        intakeIn();
        //}
        //if(gamepad1.left_trigger > .1){
        setTransferPower(-gamepad2.left_stick_y*0.8);
        //intakeOut();
        //}
        intakeMachine();
        telemetry.addData("IntakeCurent", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("transferCurent", transfer.getCurrent(CurrentUnit.AMPS));
    }

    public void updateSingle(){

        setIntPower(gamepad1.right_trigger+0.1);
        intakeIn();
        //}
        //if(gamepad1.left_trigger > .1){
        setTransferPower(gamepad1.left_trigger*0.8);
        //intakeOut();
        //}
        intakeMachine();
        telemetry.addData("IntakeCurent", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("transferCurent", transfer.getCurrent(CurrentUnit.AMPS));
    }


}
