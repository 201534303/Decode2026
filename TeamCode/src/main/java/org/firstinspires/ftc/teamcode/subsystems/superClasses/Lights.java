package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lights {
    private Servo ballLight;
    private Servo indicatorLight;
    private ElapsedTime timer;
    private double bTime;
    private double iTime;


    public Lights(ElapsedTime t, HardwareMap hardwareMap){
        ballLight = hardwareMap.get(Servo.class, "turret_right");
        indicatorLight = hardwareMap.get(Servo.class, "turret_left");
        timer = t;
    }

    public void setBallLight(double color, double b){

    }
    public void setIndicatorLight(double color, double i){

    }


}
