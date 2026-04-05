package org.firstinspires.ftc.teamcode.subsystems.superClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feedback {
    public static final double NO_BALL_RED = 0.28;
    public static final double ALLIANCE_BLUE = 0.63;
    public static final double ALLIANCE_RED = NO_BALL_RED;
    public static final double RELOCALIZED_BLUE = 0.60;
    public static final double WHEELS_LOCKED_PURPLE = 0.72;
    public static final double HAVE_BALL_GREEN = 0.5;
    private static final int RUMBLE_DURATION_MS = 500;
    private static final double RELOCALIZED_DURATION_MS = 500.0;

    private final Servo indicatorLight;
    private final ElapsedTime timer;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private boolean haveBall = false;
    private boolean wheelsLocked = false;
    private double relocalizedUntilMs = Double.NEGATIVE_INFINITY;

    public Feedback(HardwareMap hardwareMap, ElapsedTime t, Telemetry te) {
        this(hardwareMap, t, te, null, null);
    }

    public Feedback(HardwareMap hardwareMap, ElapsedTime t, Telemetry te, Gamepad g1, Gamepad g2) {
        indicatorLight = hardwareMap.get(Servo.class, "taillight");
        timer = t;
        telemetry = te;
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void notifyRelocalized() {
        relocalizedUntilMs = timer.milliseconds() + RELOCALIZED_DURATION_MS;
        if (gamepad1 != null) {
            gamepad1.rumble(RUMBLE_DURATION_MS);
        }
        refreshIndicatorLight();
    }

    public void updateBallState(boolean haveBall) {
        if (haveBall && !this.haveBall) {
            if (gamepad1 != null) {
                gamepad1.rumble(RUMBLE_DURATION_MS);
            }
            if (gamepad2 != null) {
                gamepad2.rumble(RUMBLE_DURATION_MS);
            }
        }
        this.haveBall = haveBall;
        refreshIndicatorLight();
        telemetry.addData("feedbackHaveBall", haveBall);
    }

    public void updateWheelLockState(boolean wheelsLocked) {
        this.wheelsLocked = wheelsLocked;
        refreshIndicatorLight();
        telemetry.addData("feedbackWheelLocked", wheelsLocked);
    }

    public void showAllianceSelection(boolean isBlueAlliance) {
        indicatorLight.setPosition(isBlueAlliance ? ALLIANCE_BLUE : ALLIANCE_RED);
    }

    private void refreshIndicatorLight() {
        double indicatorPosition = haveBall ? HAVE_BALL_GREEN : NO_BALL_RED;

        if (wheelsLocked) {
            indicatorPosition = WHEELS_LOCKED_PURPLE;
        }
        if (timer.milliseconds() < relocalizedUntilMs) {
            indicatorPosition = RELOCALIZED_BLUE;
        }

        indicatorLight.setPosition(indicatorPosition);
    }
}
