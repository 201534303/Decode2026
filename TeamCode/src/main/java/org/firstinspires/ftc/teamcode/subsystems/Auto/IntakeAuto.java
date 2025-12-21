package org.firstinspires.ftc.teamcode.subsystems.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.supperClasses.Intake;

public class IntakeAuto extends Intake {
    public IntakeAuto(HardwareMap hardwareMap, Telemetry t) {
        super(hardwareMap, t);
    }

    @Override
    public void intakeIn(){ intake.setPower(1); }
    @Override
    public void intakeOut(){ intake.setPower(-1); }
    @Override
    public void intakeOff(){ intake.setPower(0); }
    @Override
    public void setTransferPower(double power){
        transfer.setPower(power);
    }

    public void allTheWay(){
        intakeIn();
        setTransferPower(0.9);
    }

    public void setIntakeSpeed(double power){
        intake.setPower(power);
    }

    public void transferOff(){
        setTransferPower(0);
    }

}
