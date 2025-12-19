package org.firstinspires.ftc.teamcode.subsystems.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterAuto extends Shooter {

    public ShooterAuto(HardwareMap hardwareMap, Telemetry t, ElapsedTime r) {
        super(hardwareMap, t, r);
    }

    public void runFlywheel(double currentV, double targetV, double kf){
        flywheelSpin(targetV, currentV, kf);
    }

    public void close(){
        runFlywheel(getMotorVel(), 1250, 0);//1300
    }
    public void far(){
        runFlywheel(getMotorVel(), 1610, 0);
    }

    public void off(){
        runFlywheel(getMotorVel(), 0, 0);
    }

}
