package org.firstinspires.ftc.teamcode.subsystems.Auto;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;

public class IntakeAuto extends Intake {
    public IntakeAuto(HardwareMap hardwareMap, Telemetry t, ElapsedTime e) {
        super(hardwareMap, t, e);
    }

    @Override
    public void intakeIn(){ intake.setPower(1); }
    @Override
    public void intakeOut(){ intake.setPower(-1); }
    @Override
    public void intakeOff(){ intake.setPower(0); }

    public void off(){
        intake.setPower(0);
        transfer.setVelocity(0);
    }

    public void allTheWay(){
        intakeIn();
        setTransferPower(0.9);
    }

    public void allTheWaySlow(){
        setIntakeSpeed(0.9);
        setTransferPower(0.65);
    }

    public void setIntakeSpeed(double power){
        intake.setPower(power);
    }

    public void transferOff(){
        setTransferPower(0);
    }
}
