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
    public void setPower(double p){
        super.setPower(p);
    }

    public void update() {
        if (gamepad1.a) {
            setSpeed(55);
        }
        if (gamepad1.b) {
            setSpeed(0);
        }
        updateRoot();
        telemetry.addData("We are here", "yeah");
        rotSpeed();
    }

    public void updateSimple() {
        if (gamepad1.a) {
            setPower(0.5);
        }
        if (gamepad1.b) {
            setPower(0);
        }
        telemetry.addData("We are here", "yeah");
    }
}

