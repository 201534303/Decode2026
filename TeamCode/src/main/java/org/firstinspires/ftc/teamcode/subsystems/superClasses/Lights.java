package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Lights {
    private final Servo indicatorLight;
    private final ElapsedTime timer;
    private final Telemetry telemetry;
    private double[] baseColors = {0};
    private double baseStepMs = 1000; // time between changing colors
    private int baseIndex = 0;

    private double[] advColors = {0};
    private double advStepMs = 1000;// time between changing colors

    private double advDelayMs = 30000;    // wait before starting
    private double advDurationMs = 2000;  // how long it runs once it starts
    private double advTime = 0;

    private int advIndex = 0;
    private double lastTime = 0;

    public Lights(HardwareMap hardwareMap, ElapsedTime t, Telemetry te) {
        indicatorLight = hardwareMap.get(Servo.class, "taillight");
        timer = t;
        telemetry = te;

        double now = timer.milliseconds();
        lastTime = now;
    }

    public void setIndicatorLight(double[] colors, double stepMs) {
        if (!Arrays.equals(baseColors, colors) || baseStepMs != stepMs) {
            baseColors = colors;
            baseStepMs = stepMs;
            baseIndex = 0;
            lastTime = timer.milliseconds();
        }
    }

    public void setIndicatorLightAdvance(double[] colors, double stepMs, double delayMs, double durationMs){
        if (colors == null || colors.length == 0) return;
        advColors = colors;

        advStepMs = stepMs; //how long it takes to switch between colors
        advDelayMs = delayMs; //
        advDurationMs = durationMs; //how long it runs

        advTime = timer.milliseconds();

    }


    public void update() {
        double now = timer.milliseconds();

        double[] use = {0.0};
        double useTime = 0.0;

        if(now - advTime > advDelayMs && now - advTime  < advDelayMs + advDurationMs) {
            use = advColors;
            useTime = advStepMs;
            telemetry.addData("going?", "true");
        }
        else{
            use = baseColors;
            useTime = baseStepMs;
        }

        if(now - lastTime > useTime){
            baseIndex ++;
            lastTime = now;
        }
        if(baseIndex >= use.length) {
            baseIndex = 0;
        }

        indicatorLight.setPosition(use[baseIndex]);
    }
}
