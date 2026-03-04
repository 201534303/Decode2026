package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;

public class LimelightV5 {

    public final LimelightPose pose = new LimelightPose();
    private final double KNOWN_ANGLE = 38;
    private static final double CONSTX = 17.0;
    private static final double CONSTY = 14.375;
    private static final double FIELD_LENGTH = 144.0;
    private final Limelight3A limelight;
    public ArrayList<Double> yaws = new ArrayList<>();


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
    public void updatePos(double headingIn){
        double heading = headingIn;
        double id = pose.id;
        double tx = pose.tx;
        double distance = pose.distance;
        if (id == 20) {
            heading = 180 - heading;
        }
        double theta = Math.toRadians(heading - tx);
        pose.theta = theta;
        double camX = distance * Math.cos(theta);
        double camY = distance * Math.sin(theta);

        double dx = 10*Math.cos(Math.toRadians(heading));
        double dy = 10*Math.sin(Math.toRadians(heading));
        pose.dx = dx;
        pose.dy = dy;
        pose.rawX = camX + dx;
        pose.rawY = camY + dy;

        if (id == 20) {
            pose.posX = pose.rawX + CONSTX;
            pose.posY = FIELD_LENGTH - pose.rawY - CONSTY;
        } else { // id == 24
            pose.posX = FIELD_LENGTH - pose.rawX - CONSTX;
            pose.posY = FIELD_LENGTH - pose.rawY - CONSTY;
        }
    }
    public ArrayList<Object[]> updateBall2() {
        LLResult result = limelight.getLatestResult();
        ArrayList<Object[]> detections = new ArrayList<>();
        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            double ty = detection.getTargetYDegrees();
            double tx = detection.getTargetXDegrees();
            double camZ = 7.5/Math.tan(Math.toRadians(ty));
            double camX = camZ*Math.tan(Math.toRadians(tx));
            double classId = detection.getClassId();
            String className = detection.getClassName();
            double confidence = detection.getConfidence();
            Object[] ret = {camX, classId, className, confidence};
            detections.add(ret);
        }
        return detections;
    }
    public void updateHeading(boolean movingOrRotating) {
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
            double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
            distance *= 0.95;
            if (!movingOrRotating) {
                yaws.add(yaw);
                Collections.sort(yaws);
                int middle = yaws.size() / 2;
                int size = yaws.size();
                pose.roll = size;
                if (size % 2 == 1) {
                    pose.median_yaw = yaws.get(middle);
                }
                else {
                    pose.median_yaw = (yaws.get(middle) + yaws.get(middle-1)) / 2;
                }
            }
            else {
                yaws.clear();
                pose.median_yaw = 0;
            }
            double heading = 0;
            if (pose.median_yaw != 0) {
                heading = KNOWN_ANGLE + Math.abs(pose.median_yaw);
            }
            else {
                heading = KNOWN_ANGLE + Math.abs(yaw);
            }
            if (id == 20) {
                heading = 180 - heading;
            }
            double theta = Math.toRadians(heading - tx);
            pose.yaw = yaw;
            pose.heading = heading;
            pose.tx = tx;
            pose.distance = distance;
            pose.theta = theta;
            pose.id = id;
            pose.valid = true;
            //telemetry.addData("X (cos):", distance*Math.cos(Math.toRadians(theta)));
            //telemetry.addData("Z (sin):",  distance*Math.sin(Math.toRadians(theta)));
        }
    }
}
