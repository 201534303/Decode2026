package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

import java.util.Arrays;
import java.util.List;

public class LimelightProcessor_v3Tele {

    public final LimelightPose pose = new LimelightPose();
    private final Limelight3A limelight;

    private static final double CONSTX = 16.0;
    private static final double CONSTY = 13.375;
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

        pose.pitch = camRot.getPitch();
        pose.roll  = camRot.getRoll();

        // Raw distance in inches
        double rawDistance = Math.sqrt(camX * camX + camZ * camZ) * 39.3701 + 6.0;

        // ================= DISTANCE EMA =================
        if (!pose.valid) {
            pose.distance = rawDistance; // initialize
        } else {
            pose.distance = DIST_ALPHA * pose.distance + (1 - DIST_ALPHA) * rawDistance;
        }

        // ================= TX (MEDIAN FILTER) =================
        double rawTx = Math.toRadians(fiducial.getTargetXDegrees());
        double txMedian = medianTx(rawTx);

        // Deadband
        if (Math.abs(txMedian) < TX_DEADBAND) txMedian = 0.0;

        // Nonlinear mapping
        if (Math.abs(Math.toDegrees(txMedian)) > NONLINEAR_THRESHOLD_DEG) {
            double deg = Math.abs(Math.toDegrees(txMedian));
            txMedian = Math.toRadians(0.0001 * Math.pow(deg, 4.53058));
        }

        pose.tx = txMedian;

        // ================= ANGLES =================
        double yaw = Math.toRadians(yawDegIn);
        if (yaw > Math.toRadians(90)) yaw = Math.toRadians(180) - yaw;

        double shooter = Math.toRadians(shooterDegIn);

        double thetaRaw = yaw + shooter + Math.toRadians(2.0) - txMedian;

        // Filter theta only if stationary
        if (!pose.valid || !moving) {
            final double THETA_ALPHA = 0.75;
            pose.theta = THETA_ALPHA * pose.theta + (1 - THETA_ALPHA) * thetaRaw;
        } else {
            pose.theta = thetaRaw;
        }

        double theta = pose.theta;
        pose.heading = yaw;

        // ================= FIELD POSITION =================
        pose.rawX = pose.distance * Math.cos(theta);
        pose.rawY = pose.distance * Math.sin(theta);

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

    // ------------------------------------------------------------
    // MEDIAN FILTER IMPLEMENTATION
    // ------------------------------------------------------------
    private double medianTx(double newTx) {
        txBuf[txIdx] = newTx;
        txIdx = (txIdx + 1) % MEDIAN_SIZE;

        if (txIdx == 0) txFilled = true;
        if (!txFilled) return newTx; // warm-up

        double[] copy = txBuf.clone();
        Arrays.sort(copy);
        return copy[MEDIAN_SIZE / 2];
    }
}
