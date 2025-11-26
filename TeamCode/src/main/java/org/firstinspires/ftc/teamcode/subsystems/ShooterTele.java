package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterTele extends Shooter{
    private Gamepad gamepad1, gamepad2;
    private double speed = 0;

    public ShooterTele(HardwareMap h, Gamepad g1, Gamepad g2, Telemetry t, ElapsedTime r) {
        super(h, t, r);
        gamepad1 = g1;
        gamepad2 = g2;
    }
    public void setVel(double flywheelV){
        super.setVel(flywheelV);
    }
/*
    public void update() {
        updateRoot();
        telemetry.addData("We are here", "yeah");
        rotSpeed();
    }

 */

    public void runFlywheel(double currentV, double targetV){
        if (gamepad2.a) {
            flywheelSpin(targetV, currentV);
        }
        if (gamepad2.b) {
            flywheelSpin(0, currentV);
        }

    }

    public void updateSimple() {
        if (gamepad2.a) {
            setVel(0.5);
        }
        if (gamepad2.b) {
            setVel(0);
        }
        telemetry.addData("We are here", "yeah");
    }
}

