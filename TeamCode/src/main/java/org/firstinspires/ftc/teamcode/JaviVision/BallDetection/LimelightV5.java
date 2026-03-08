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
    private ArrayList<Double[]> oldPurpleBalls = new ArrayList<>();
    private ArrayList<Double[]> oldGreenBalls = new ArrayList<>();
    public BallTracker trackerGreen;
    public BallTracker trackerPurple;


    public LimelightV5(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // update Ball pipeline
        //limelight.pipelineSwitch(4); // HSVTuner pipeline
        limelight.start();
        trackerGreen = new BallTracker();
        trackerPurple = new BallTracker();
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
    public ArrayList<double[]> updateBall2(double timeDif) {
        LLResult result = limelight.getLatestResult();

        /*double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = (nowTime - lastTime);
        lastTime = nowTime;*/

        ArrayList<double[]> newPurpleBalls = new ArrayList<>();
        ArrayList<double[]> newGreenBalls = new ArrayList<>();

        ArrayList<double[]> detections = new ArrayList<>();
        ArrayList<double[]> output = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            double ty = detection.getTargetYDegrees();
            double tx = detection.getTargetXDegrees();
            double distance = 5/Math.sqrt(detection.getTargetArea());
            double camZ = 6.5/Math.tan(Math.toRadians(ty));
            double camX = camZ*Math.tan(Math.toRadians(tx));
            int classId = detection.getClassId();
            String className = detection.getClassName();
            double confidence = detection.getConfidence();

            double[] retList = {camX, camZ, 1, 0, 0};
            double[] retForTest = {camX, camZ};

            if (className.equals("green")) {
                if (confidence >= greenLowerConf) {
                    double[] ret = {camX, camZ, (double) classId, 0, confidence, distance};
                    newGreenBalls.add(retForTest);
                    //output.add(retList);
                }
            }
            else if (className.equals("purple")) {
                if (confidence >= purpleLowerConf) {
                    double[] ret = {camX, camZ, (double) classId, 1, confidence, distance};
                    detections.add(ret);
                    newPurpleBalls.add(retForTest);
                    //output.add(retList);
                }
            }// once at init

            // timeDif in seconds

        }
        trackerGreen.update(newGreenBalls, timeDif);
        trackerPurple.update(newPurpleBalls, timeDif);

// temp debug — add a fake entry with timeDif info
        ArrayList<double[]> outputG = new ArrayList<>();
        ArrayList<double[]> outputP = new ArrayList<>();

        for (TrackedBall ball : trackerGreen.getAliveBalls()) {
            double[] r = {ball.x, ball.y, ball.vx, ball.vy, ball.id};
            outputG.add(r);
            // ball.x, ball.y  → position
            // ball.vx, ball.vy → velocity in units/sec
        }
        for (TrackedBall ball : trackerPurple.getAliveBalls()) {
            double[] r = {ball.x, ball.y, ball.vx, ball.vy, ball.id};
            outputP.add(r);
        }
        output.addAll(outputG);
        output.addAll(outputP);
        return detections;
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
