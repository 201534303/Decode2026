package org.firstinspires.ftc.teamcode.JaviVision.v2;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

import java.util.List;

public class LimelightProcessor_v2 {

    public final LimelightPose pose = new LimelightPose();
    private static final double CAMERA_HEIGHT = 11.5;   // inches
    private static final double TARGET_HEIGHT = 29.4375;   // inches
    private static final double CAMERA_ANGLE = 0.0;    // degrees upward tilt
    private double tx;
    private double ty;
    private double ta;
    private double distance;
    private int id;
    private Limelight3A limelight;

    public LimelightProcessor_v2(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();
    }
    
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult tag = tags.get(0); // take the first detected tag

                    id = tag.getFiducialId();

                    tx = tag.getTargetXDegrees();       // horizontal deg
                    ty = tag.getTargetYDegrees();       // vertical deg
                    ta = tag.getTargetArea();       // area
                if (id != 0) {
                    // ---- DISTANCE CALC ----
                    double angleToTag = CAMERA_ANGLE + ty;
                    double angleRad = Math.toRadians(angleToTag);

                    double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
                    distance = heightDiff / Math.tan(angleRad);

                    double error = 0.281091 * distance - 10.70099;
                    distance += error;
                    pose.valid = true;
                }
                else {
                    pose.valid = false;
                }
            }
            else {
                pose.valid = false;
            }

        }

        pose.tx = tx;
        pose.ty = ty;
        pose.tz = 0;

        pose.rx = 0;
        pose.ry = 0;
        pose.rz = 0;

        pose.distance = distance;

        // Correction
        double error = -0.0569126 * pose.distance + 3.85921;
        pose.distance += error;

        pose.id = id;
    }
}
