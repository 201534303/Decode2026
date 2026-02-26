package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;

public class LimelightV5 {

    public final LimelightPose pose = new LimelightPose();
    public final double KNOWN_ANGLE = 42;
    private final Limelight3A limelight;

    public LimelightV5(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // update Ball pipeline
        //limelight.pipelineSwitch(4); // HSVTuner pipeline
        limelight.start();
    }

    // ------------------------------------------------------------
    public double[] updateBall() {
        // ================= LIMELIGHT =================
        double[] purple = {164, 13, 84, 255, 5, 58};
        double purpleH = 100000 + purple[0]*1000 + purple[1];
        double purpleS = 100000 + purple[2]*1000 + purple[3];
        double purpleV = 100000 + purple[4]*1000 + purple[5];
        double[] green = {55,75,125,255,5,71};
        double greenH = 100000 + green[0]*1000 + green[1];
        double greenS = 100000 + green[2]*1000 + green[3];
        double greenV = 100000 + green[4]*1000 + green[5];
        double[] inputs = {1 , purpleH, purpleS, purpleV, 1, greenH, greenS, greenV};
        //             P, Hmin, Smin, Vmin, Hmax, Smax, vmax, 0
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
    public double[] updateHSV() {
        // ================= LIMELIGHT =================
        double[] inputs = {1, 0, 0, 0, 0, 0, 0, 0};
        //                 P, Hmin, Smin, Vmin, Hmax, Smax, vmax, 0
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
    public void updatePos(double headingIn, boolean updateHeading) {
        double[] results = limelight.getLatestResult().getPythonOutput();
        if (results[0] == 0) {
            pose.valid = false;
        }
        else if (results[0] == 1) {
            double x = results[1];
            double y = results[2];
            double z = results[3];
            double yaw = results[4];
            double tx = results[6];
            int id = (int) results[7];
            double heading = 0;
            if (updateHeading) {
                heading = Math.toDegrees(headingIn);
            }
            else {
                heading = KNOWN_ANGLE - yaw;
            }
            double theta = Math.toRadians(heading - tx);
            double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
            double posX = distance * Math.cos(theta);
            double posY = distance * Math.sin(theta);
            pose.yaw = yaw;
            pose.heading = heading;
            pose.tx = tx;
            pose.distance = distance;
            pose.theta = theta;
            pose.posX = posX;
            pose.posY = posY;
            pose.id = id;
            //telemetry.addData("X (cos):", distance*Math.cos(Math.toRadians(theta)));
            //telemetry.addData("Z (sin):",  distance*Math.sin(Math.toRadians(theta)));
        }
    }
}
