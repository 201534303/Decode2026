package org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV4;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;

import java.util.ArrayList;

public class LimelightProcessor_v4Tele {

    public final LimelightPose pose = new LimelightPose();
    private final Limelight3A limelight;

    private static final double CONSTX = 17.0;
    private static final double CONSTY = 14.375;
    private static final double FIELD_LENGTH = 144.0;

    // ===== FILTER CONSTANTS =====
    private static final int MEDIAN_SIZE = 5;

    public LimelightProcessor_v4Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    // ------------------------------------------------------------
    public void updateTele(double imuYawDeg, double x, boolean y) {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            pose.valid = false;
            return;
        }

        // --- Get MetaTag2 robot pose (FIELD SPACE) ---
        Pose3D botPose = result.getBotpose_MT2();
        if (botPose == null) {
            pose.valid = false;
            return;
        }

        // --- Position (meters -> inches) ---
        Position pos = botPose.getPosition();
        pose.posX = pos.x * 39.3701; // might have to add 72
        pose.posY = pos.y * 39.3701; // might have to add 72

        // --- Orientation ---
        YawPitchRollAngles rot = botPose.getOrientation();
        double mtYawRad = Math.toRadians(rot.getYaw());

        // --- Optional: IMU fusion (recommended) ---
        double imuYawRad = Math.toRadians(imuYawDeg);

        // Use MT2 yaw only when robot is stable
        pose.yaw = pose.heading;
        pose.pitch = Math.toRadians(rot.getPitch());
        pose.roll  = Math.toRadians(rot.getRoll());
        // --- Validity ---
        pose.valid = true;
    }

}
