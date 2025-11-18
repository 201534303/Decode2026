package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterTele extends Shooter{
    public ShooterTele(HardwareMap h, Telemetry t) {
        super(h, t);
    }
    public void setPower(double p){
        super.setPower(p);
    }
}
