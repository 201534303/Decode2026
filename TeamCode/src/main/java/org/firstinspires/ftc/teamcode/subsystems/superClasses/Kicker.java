package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Kicker {
    Servo kickstand;

    public Kicker(HardwareMap hardwareMap, Telemetry t, ElapsedTime e){
        kickstand = hardwareMap.get(Servo.class, "kicker");

    }

    public void kickUp(){
        kickstand.setPosition(1);
    }

    public void kickDown(){
        kickstand.setPosition(0);
    }
}
