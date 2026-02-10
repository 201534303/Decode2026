package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Lights {
    private final Servo indicatorLight;
    private final ElapsedTime timer;
    @SuppressWarnings("unused")
    private final Telemetry telemetry;

    // ---------- Base pattern ----------
    private double[] baseColors = {0.28};
    private double baseStepMs = 700;

    private int baseIndex = 0;
    private double baseLastStepMs = 0;

    // ---------- Override (advance reminder) ----------
    private double[] advColors = {0.28, 0.60};
    private double advStepMs = 300;

    private boolean advArmed = false;     // scheduled but not yet active
    private boolean advActive = false;    // currently overriding
    private double advArmTimeMs = 0;      // when scheduled
    private double advDelayMs = 30000;    // wait before starting
    private double advDurationMs = 2000;  // how long it runs once it starts

    private int advIndex = 0;
    private double advLastStepMs = 0;

    public Lights(HardwareMap hardwareMap, ElapsedTime t, Telemetry te) {
        indicatorLight = hardwareMap.get(Servo.class, "taillight");
        timer = t;
        telemetry = te;

        double now = timer.milliseconds();
        baseLastStepMs = now;
        advLastStepMs = now;
    }

    // Set the normal / always-running pattern
    public void setIndicatorLight(double[] colors, double stepMs) {
        if (colors == null || colors.length == 0) return;
        if (!Arrays.equals(baseColors, colors) || baseStepMs != stepMs) {
            baseColors = colors;
            baseStepMs = stepMs;
            baseIndex = 0;
            baseLastStepMs = timer.milliseconds();
        }
    }

    /**
     * Schedule a reminder override:
     * after delayMs, run the adv pattern for durationMs, then return to base.
     * Calling again RESTARTS the schedule cleanly.
     */
    public void scheduleRelocalizeReminder(double[] colors, double stepMs, double delayMs, double durationMs) {
        if (colors == null || colors.length == 0) return;

        advColors = colors;
        advStepMs = stepMs;
        advDelayMs = delayMs;
        advDurationMs = durationMs;

        // Re-arm cleanly every time
        advArmed = true;
        advActive = false;
        advArmTimeMs = timer.milliseconds();

        advIndex = 0;
        advLastStepMs = advArmTimeMs;
    }

    public void cancelReminder() {
        advArmed = false;
        advActive = false;
    }

    public void update() {
        double now = timer.milliseconds();

        // Manage reminder state
        if (advArmed) {
            double sinceArm = now - advArmTimeMs;

            // Start override after delay
            if (!advActive && sinceArm >= advDelayMs) {
                advActive = true;
                advIndex = 0;
                advLastStepMs = now;
            }

            // Stop override after duration
            if (advActive && sinceArm >= advDelayMs + advDurationMs) {
                advActive = false;
                advArmed = false; // one-shot
            }
        }

        // Choose which pattern to show
        if (advActive) {
            // step adv
            if (advStepMs > 0 && now - advLastStepMs >= advStepMs) {
                advLastStepMs = now;
                advIndex = (advIndex + 1) % advColors.length;
            }
            indicatorLight.setPosition(advColors[advIndex]);
        } else {
            // step base
            if (baseStepMs > 0 && now - baseLastStepMs >= baseStepMs) {
                baseLastStepMs = now;
                baseIndex = (baseIndex + 1) % baseColors.length;
            }
            indicatorLight.setPosition(baseColors[baseIndex]);
        }
    }
}
