package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

import java.util.List;
import java.util.Map;

public class LimelightProcessor_v3 {

    // ===== PUBLIC STATE =====
    public final LimelightPose pose = new LimelightPose();
    private Limelight3A limelight;

    // ===== TAG FIELD LOCATIONS (METERS) =====
    private static class TagPose {
        double x, y, yaw;
        TagPose(double x, double y, double yaw) {
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }

    private final Map<Integer, TagPose> tagMap = Map.of(
            24, new TagPose(3.2385, 3.305, Math.toRadians(180)),
            20, new TagPose(0.381, 3.305, Math.toRadians(0))
    );

    // ===== INIT =====
    public LimelightProcessor_v3(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    // ===== UPDATE VISION DATA =====
    public void update() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            pose.valid = false;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            pose.valid = false;
            return;
        }

        LLResultTypes.FiducialResult fiducial = fiducials.get(0);
        Pose3D camPose = fiducial.getCameraPoseTargetSpace();

        Position position = camPose.getPosition();
        YawPitchRollAngles rotation = camPose.getOrientation();

        double camX = position.x;
        double camY = position.y;
        double camZ = position.z;

        double camYaw = rotation.getYaw();
        double camPitch = rotation.getPitch();
        double camRoll = rotation.getRoll();

        if (Math.abs(camYaw) > Math.toRadians(90)) {
            camX = -camX;
            camY = -camY;
            camYaw -= Math.copySign(Math.PI, camYaw);
        }

        pose.x = camX;
        pose.y = camY;
        pose.z = camZ;

        pose.yaw = camYaw;
        pose.pitch = camPitch;
        pose.roll = camRoll;

        pose.distance = Math.sqrt(camX*camX + camZ * camZ);

        pose.id = fiducial.getFiducialId();
        pose.valid = true;
    }

    // ===== FIELD POSITION SOLVER =====
    public void getRobotPose() {

        if (!pose.valid) return;

        TagPose tag = tagMap.get(pose.id);
        if (tag == null) return;

        // Camera → Tag
        double[][] T_cam_tag = buildTransform(
                pose.x, 0, pose.z,
                0, pose.yaw, 0
        );

        // Tag → Camera
        double[][] T_tag_cam = invert(T_cam_tag);

        // Field → Tag
        double[][] T_field_tag = buildTransform(
                tag.x, 0, tag.y,
                0, tag.yaw, 0
        );

        // Field → Camera
        double[][] T_field_cam = multiply(T_field_tag, T_tag_cam);

        // Save final field position
        pose.posX = T_field_cam[0][3];
        pose.posY = T_field_cam[2][3];
    }

    // ===== TRANSFORM MATH =====

    static double[][] buildTransform(double x, double y, double z,
                                     double roll, double yaw, double pitch)
    {
        double cy = Math.cos(yaw), sy = Math.sin(yaw);

        return new double[][]{
                { cy, 0, sy, x },
                {  0, 1,  0, y },
                { -sy, 0, cy, z },
                {  0, 0,  0, 1 }
        };
    }

    static double[][] invert(double[][] T) {

        double[][] inv = new double[4][4];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                inv[i][j] = T[j][i];

        for (int i = 0; i < 3; i++)
            inv[i][3] =
                    -(inv[i][0]*T[0][3] +
                            inv[i][1]*T[1][3] +
                            inv[i][2]*T[2][3]);

        inv[3][3] = 1;
        return inv;
    }

    static double[][] multiply(double[][] A, double[][] B) {

        double[][] R = new double[4][4];

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    R[i][j] += A[i][k] * B[k][j];

        return R;
    }
}
