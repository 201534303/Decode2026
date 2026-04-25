package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private static final double greenLowerConf = 0.30;
    private static final double purpleLowerConf = 0.55;
    public ArrayList<Double> yaws = new ArrayList<>();
    public ArrayList<Double> dists = new ArrayList<>();
    private ArrayList<double[]> oldPurpleBalls = new ArrayList<>();
    private ArrayList<double[]> oldGreenBalls = new ArrayList<>();
    private ArrayList<double[]> newGreenBalls = new ArrayList<>();
    private ArrayList<double[]> newPurpleBalls = new ArrayList<>();
    private double greenVelocity;
    private double purpleVelocity;


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
    private double[] averagePos(ArrayList<double[]> input) {
        double sum_x = 0;
        double sum_y = 0;
        for (double[] ball : input) {
            sum_x += ball[0];
            sum_y += ball[1];
        }
        sum_x = sum_x / input.size();
        sum_y = sum_y / input.size();
        double[] ret = {sum_x, sum_y};
        return ret;
    }
    public ArrayList<double[]> updateBall2(double timeDif, boolean first) {
        LLResult result = limelight.getLatestResult();
        ArrayList<double[]> detections = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            double ty = detection.getTargetYDegrees();
            double tx = detection.getTargetXDegrees();
            double distance = 5/Math.sqrt(detection.getTargetArea());
            double camZ = 6.5/Math.tan(Math.toRadians(ty));
            double camX = camZ*Math.tan(Math.toRadians(tx));
            int classId = detection.getClassId();
            String className = detection.getClassName();
            double confidence = detection.getConfidence();
            if (className.equals("green")) {
                if (confidence >= greenLowerConf) {
                    double[] ret = {camX, camZ, 0, 0, (double) classId, 0};
                    detections.add(ret);
                    if (first)  {
                        oldGreenBalls.add(ret);
                    }
                    else{
                        newGreenBalls.add(ret);
                    }
                }
            }
            else if (className.equals("purple")) {
                if (confidence >= purpleLowerConf) {
                    double[] ret = {camX, camZ, 0, 0, (double) classId, 1};
                    detections.add(ret);
                    if (first)  {
                        oldPurpleBalls.add(ret);
                    }
                    else {
                        newPurpleBalls.add(ret);
                    }
                }
            }
        }
        if (!first) {
            double[] velPurple = {0, 0};
            double[] velGreen = {0, 0};

            if (!newPurpleBalls.isEmpty() && !oldPurpleBalls.isEmpty()) {
                velPurple[0] = (averagePos(newPurpleBalls)[0] - averagePos(oldPurpleBalls)[0]) / timeDif;
                velPurple[1] = (averagePos(newPurpleBalls)[1] - averagePos(oldPurpleBalls)[1]) / timeDif;
            }
            if (!newGreenBalls.isEmpty() && !oldGreenBalls.isEmpty()) {
                velGreen[0] = (averagePos(newGreenBalls)[0] - averagePos(oldGreenBalls)[0]) / timeDif;
                velGreen[1] = (averagePos(newGreenBalls)[1] - averagePos(oldGreenBalls)[1]) / timeDif;
            }

            //double[] velPurple = {(averagePos(newPurpleBalls)[0] - averagePos(oldPurpleBalls)[0])/timeDif, (averagePos(newPurpleBalls)[1] - averagePos(oldPurpleBalls)[1])/timeDif};
            //double[] velGreen = {(averagePos(newPurpleBalls)[0] - averagePos(oldGreenBalls)[0])/timeDif, (averagePos(newPurpleBalls)[1] - averagePos(oldPurpleBalls)[1])/timeDif};

            for (double[] ball : detections) {
                if (ball[5] == 0) {
                    ball[2] = velGreen[0];
                    ball[3] = velGreen[1];
                }
                else {
                    ball[2] = velPurple[0];
                    ball[3] = velPurple[1];
                }
            }
        }
        return detections;
    }

    public void clearBallLists() {
        oldGreenBalls.clear();
        oldPurpleBalls.clear();
        newGreenBalls.clear();
        newPurpleBalls.clear();
    }
    public void updateHeading(boolean movingOrRotating) {
        double[] results = limelight.getLatestResult().getPythonOutput();
        if (results[0] == 0) {
            pose.valid = false;
        }
        else if (results[0] == 1) {
            double x = results[1];
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
                dists.add(distance);
                Collections.sort(dists);
                middle = dists.size() / 2;
                size = dists.size();
                if (size % 2  == 1) {
                    pose.median_distance = dists.get(middle);
                }
                else {
                    pose.median_distance = (dists.get(middle) + dists.get(middle - 1))/2;
                }
            }
            else {
                yaws.clear();
                pose.median_yaw = 0;
                dists.clear();
                pose.median_distance = 0;
            }

            double alpha = Math.atan(x/z);
            pose.yaw = yaw;
            pose.distance = distance;
            if (pose.median_yaw != 0) {
                yaw = pose.median_yaw;
            }
            if (pose.median_distance != 0)
            {
                distance = pose.median_distance;
            }

            double theta = Math.toRadians(90 - (KNOWN_ANGLE - yaw + alpha));

            pose.heading = 90 - (KNOWN_ANGLE - yaw);

            pose.rawX = distance*Math.cos(theta);
            pose.rawY = distance*Math.sin(theta);

            pose.tx = tx;

            pose.distance = distance;
            pose.theta = Math.toDegrees(theta);

            double dx = 10*Math.cos(Math.toRadians(pose.heading));
            double dy = 10*Math.sin(Math.toRadians(pose.heading));

            pose.dx = dx;
            pose.dy = dy;
            pose.rawX += dx;
            pose.rawY += dy;

            if (id == 20) {
                pose.posX = pose.rawX + CONSTX;
                pose.posY = FIELD_LENGTH - pose.rawY - CONSTY;
            } else { // id == 24
                pose.posX = FIELD_LENGTH - pose.rawX - CONSTX;
                pose.posY = FIELD_LENGTH - pose.rawY - CONSTY;
            }

            pose.id = id;
            pose.valid = true;

        }
    }
}
