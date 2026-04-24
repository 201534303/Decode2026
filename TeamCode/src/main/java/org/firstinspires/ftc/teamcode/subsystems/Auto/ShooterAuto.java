package org.firstinspires.ftc.teamcode.subsystems.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

public class ShooterAuto extends Shooter {

    public ShooterAuto(HardwareMap hardwareMap, Telemetry t, ElapsedTime r) {
        super(hardwareMap, t, r);
    }

    public void runFlywheel(double currentV, double targetV, double kf){
        flywheelSpinBangBang(targetV, currentV, kf);
    }

    public void close(){
        //runFlywheel(getMotorVel(), 1250, 0);//1300
        runFlywheel(getMotorVel(), 1124.31822703, 0);//1266
    }
    public void closeFaster(){
        //runFlywheel(getMotorVel(), 1250, 0);//1300
        runFlywheel(getMotorVel(), 1250, 0);//1266
    }

    public void set(double speed){
        //runFlywheel(getMotorVel(), 1250, 0);//1300
        runFlywheel(getMotorVel(), speed, 0);//1266
    }

    public void closeSlow(){
        //runFlywheel(getMotorVel(), 1250, 0);//1300
        runFlywheel(getMotorVel(), 1450, 0);//1300
    }

    public void closeMove(int targetV){
        runFlywheel(getMotorVel(), targetV, 0);//1300
    }
    public void far(){
        runFlywheel(getMotorVel(), 1385, 0);//1356.85549118
    }

    public void farFaster(){
        runFlywheel(getMotorVel(), 1418.49330227, 0);//1610
    }

    public void off(){
        runFlywheel(getMotorVel(), 0, 0);
    }

}
