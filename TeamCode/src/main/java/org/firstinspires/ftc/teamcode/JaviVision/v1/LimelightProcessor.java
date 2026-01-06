package org.firstinspires.ftc.teamcode.JaviVision.v1;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

public class LimelightProcessor {

    private final Limelight3A limelight;
    public final LimelightPose pose = new LimelightPose();

    public LimelightProcessor(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        double[] py = (result != null) ? result.getPythonOutput() : null;

        if (py == null || py.length < 8 || py[7] == 0) {
            pose.valid = false;
            return;
        }

        pose.x = py[0] * 39.37;
        pose.y = py[1] * 39.37;
        pose.z = py[2] * 39.37;

        pose.yaw = py[3] * 180 / Math.PI;
        pose.pitch = py[4] * 180 / Math.PI;
        pose.roll = py[5] * 180 / Math.PI;

        pose.distance = Math.sqrt(pose.x * pose.x + pose.z * pose.z);

        // Correction
        //double error = -0.0569126 * pose.distance + 3.85921;
        double error = 0;
        pose.distance += error;
        pose.valid = true;
    }
}
