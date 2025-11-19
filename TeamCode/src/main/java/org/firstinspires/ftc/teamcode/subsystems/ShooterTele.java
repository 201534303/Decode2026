package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterTele extends Shooter{
    private Gamepad gamepad1, gamepad2;

    public ShooterTele(HardwareMap h, Gamepad g1, Gamepad g2, Telemetry t) {
        super(h, t);
        gamepad1 = g1;
        gamepad2 = g2;
    }
    public void setPower(double p){
        super.setPower(p);
    }

    public void update(){
        if(gamepad1.a){
            setPower(.55);
        }
        if(gamepad1.b){
            setPower(0);
        }
    }
}
