package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class LimelightProcessor_v3Tele {

    public final LimelightPose pose = new LimelightPose();
    private final Limelight3A limelight;

    private static final double CONSTX = 17.0;
    private static final double CONSTY = 14.375;
    private static final double FIELD_LENGTH = 144.0;

    // ===== FILTER CONSTANTS =====
    private static final int MEDIAN_SIZE = 5;
    private static final double TX_DEADBAND = Math.toRadians(0.3);
    private static final double DIST_ALPHA = 0.5;       // EMA smoothing for distance
    private static final double NONLINEAR_THRESHOLD_DEG = 3.0;

    // ===== MEDIAN FILTER STATE =====
    private final double[] txBuf = new double[MEDIAN_SIZE];
    private int txIdx = 0;
    private boolean txFilled = false;
    private ArrayList<Double> thetaList = new ArrayList<>();
    private ArrayList<Double> txList = new ArrayList<>();
    private ArrayList<Double> distanceList = new ArrayList<>();

    public LimelightProcessor_v3Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    // ------------------------------------------------------------
    public void updateTele(double yawDegIn, double shooterDegIn, boolean moving) {

        // ================= LIMELIGHT =================
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            pose.valid = false;
            return;
        }

        LLResultTypes.FiducialResult fiducial = null;
        for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
            int id = fid.getFiducialId();
            if (id == 20 || id == 24) {
                fiducial = fid;
                break;
            }
        }

        if (fiducial == null) {
            pose.valid = false;
            return;
        }

        // ================= CAMERA POSE =================
        Pose3D camPose = fiducial.getCameraPoseTargetSpace();
        Position camPos = camPose.getPosition();
        YawPitchRollAngles camRot = camPose.getOrientation();

        double camX = camPos.x;
        double camZ = camPos.z;
        double camYaw = camRot.getYaw();
        pose.x = camX;
        pose.z = camZ;
        pose.yaw = camYaw;

        pose.pitch = camRot.getPitch();
        pose.roll  = camRot.getRoll();

        // Raw distance in inches
        double rawDistance = Math.sqrt(camX * camX + camZ * camZ) * 39.3701 + 6.0;
        if (!moving) {
            distanceList.add(rawDistance);
            Collections.sort(distanceList);
        } else {
            distanceList.clear();
            distanceList.add(rawDistance);
        }
        double medianDistance = 0;
        if (!distanceList.isEmpty()) {
            medianDistance = distanceList.get(distanceList.size() / 2);
        }

        pose.distance = medianDistance;

        // ================= TX (MEDIAN FILTER) =================
        double rawTx = fiducial.getTargetXDegrees();

        /*if (!moving) {
            txList.add(rawTx);
            Collections.sort(txList);
        } else {
            txList.clear();
            txList.add(rawTx);
        }
        double medianTx = 0;
        if (txList.size() > 0) {
            medianTx = txList.get(txList.size()/2);
        }*/
        double tx = 0;
        if (rawTx < 0) {
            tx = -0.85*Math.pow(-rawTx, 1.04);
        }
        else {
            tx = 0.85*Math.pow(rawTx, 1.04);
        }
        if (pose.id == 20) {
            tx *= -1;
            tx -= 2;
        }
        else if (pose.id == 24) {
            tx += 1;
        }
        pose.tx = tx;

        // ================= ANGLES =================
        double yaw = yawDegIn;
        if (pose.id == 20) { yaw = Math.toRadians(180) - yaw; }

        double shooter = Math.toRadians(shooterDegIn);
        if (pose.id == 20) shooter = -shooter;

        double rawTheta = yaw + shooter - Math.toRadians(tx);
        // Filter theta only if stationary
        if (!moving) {
            thetaList.add(rawTheta);
            Collections.sort(thetaList);
        } else {
            thetaList.clear();
            thetaList.add(rawTheta);
        }
        double medianTheta = 0;
        if (thetaList.size() > 0) {
            medianTheta = thetaList.get(thetaList.size() / 2);
        }
        pose.theta = medianTheta;
        pose.heading = rawTheta;

        // ================= FIELD POSITION =================
        pose.rawX = pose.distance * Math.cos(pose.theta);
        pose.rawY = pose.distance * Math.sin(pose.theta);
        int id = fiducial.getFiducialId();
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
