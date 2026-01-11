package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.List;


public class LimelightProcessor_v3Tele {

    public final LimelightPose pose = new LimelightPose();
    public GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private final double CONSTX = 16;
    private final double CONSTY = 13.375;
    private final double fieldLength = 144;

    private final double alpha = 0.25;


    public LimelightProcessor_v3Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }


    public void updateTele(boolean moving) {
        //odo.update();
        //Pose2D pos = odo.getPosition();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult fiducial = null;
            if (fiducials != null && fiducials.size() > 0) {
                for (LLResultTypes.FiducialResult fid : fiducials) {
                    int id = fid.getFiducialId();
                    // making sure it doesn't see the pattern april tags
                    if ((id == 20) || (id == 24)) {
                        fiducial = fid;
                        break;
                    }
                }
                if (fiducial != null) {
                    int id = fiducial.getFiducialId();
                    Pose3D camPose = fiducial.getCameraPoseTargetSpace();
                    Position position = camPose.getPosition();
                    YawPitchRollAngles rotation = camPose.getOrientation();

                    // In the current FTC Limelight SDK, you access fields directly:
                    double camX = position.x;
                    double camY = position.y;
                    double camZ = position.z;
                    double camRoll = rotation.getRoll();
                    double camPitch = rotation.getPitch();
                    double distance = Math.sqrt(camX * camX + camZ * camZ) * 39.3701;
                    if (camY < 0) {
                        pose.x = -camX;
                        pose.y = -camY;
                    } else {
                        pose.x = camX;
                        pose.y = camY;
                    }
                    pose.z = camZ;
                    pose.pitch = camPitch;
                    pose.roll = camRoll;
                    double tx = Math.toRadians(fiducial.getTargetXDegrees());
                    if (!moving) {
                        pose.tx = alpha*pose.tx + (1-alpha)*tx;
                        pose.distance = pose.distance*alpha + (1-alpha)*(distance+6);
                        pose.y = tx;
                        pose.z = distance + 6;
                    }
                    else {
                        pose.tx = tx;
                        pose.distance = distance;
                    }
                    pose.id = id;
                    pose.valid = true;
                }
            }
            else {
                pose.valid = false;
            }
        }
        else {
            pose.valid = false;
        }
    }

    public void getRobotPose(double yawIn, double shooterIn, double txIn, int id, boolean moving) {
        double yaw = yawIn;
        double tx = 0;
        // CHECK FOR BLUE
        if (yaw > 90) {
            yaw = 180 - yaw;
        }
        if (Math.toDegrees(txIn) > 3) {
            tx = 0.0001 * Math.pow(Math.toDegrees(txIn), 4.53058);
        }

        double theta = Math.abs(yaw + Math.toRadians(shooterIn) + Math.toRadians(2) - Math.toRadians(tx));
        if (!moving) {
            pose.theta = pose.theta * alpha + (1 - alpha) * theta;
            pose.x = theta;
        }
        else {
            pose.theta = theta;
        }
        pose.heading = yaw;
        double rawX = (pose.distance)*Math.cos(theta);
        double rawY = (pose.distance)*Math.sin(theta);
        /*if (!moving) {
            pose.rawX = pose.rawX*alpha + (1-alpha)*rawX;
            pose.rawY = pose.rawY*alpha + (1-alpha)*rawY;
        }
        else {
            pose.rawX = rawX;
            pose.rawY = rawY;
        }*/
        if (id == 20) {
            pose.posX = pose.rawX + CONSTX;
            pose.posY = fieldLength - pose.rawY - CONSTY;
        }
        else if (id == 24) {
            pose.posX = fieldLength - pose.rawX - CONSTX;
            pose.posY = fieldLength - pose.rawY - CONSTY;
        }
        // UPDATE FOR FIELD COORDS
    }
    /*
    CHATGPT FUNCTINO THAT I SHOULD PUT IN ONCE I FIGURE OUT HOW LONG I CAN STAY STILL FOR
    public void getRobotPose(double yawIn, double shooterIn, double delAngle, int id) {

        double yaw = yawIn;

        // Alliance normalization
        if (yaw > 90) {
            yaw = 180 - yaw;
        }

        // ---- TX FILTERING ----
        filtered_tx = 0.85 * filtered_tx + 0.15 * stored_tx;

        double txUsed = filtered_tx;

        // Deadband small jitter
        if (Math.abs(txUsed) < Math.toRadians(0.4)) {
            txUsed = 0;
        }

        // ---- THETA CALC ----
        double rawTheta = Math.abs(
                yaw
                        + Math.toRadians(shooterIn)
                        - txUsed
                        + Math.toRadians(0.8)
        );

        // Rate limit
        double delta = rawTheta - lastTheta;
        delta = Math.max(-Math.toRadians(1.5), Math.min(Math.toRadians(1.5), delta));
        double theta = lastTheta + delta;
        lastTheta = theta;

        pose.theta = theta;
        pose.heading = yaw;

        double rawX = (6.5 + pose.distance) * Math.cos(theta);
        double rawY = (6.5 + pose.distance) * Math.sin(theta);

        pose.rawX = rawX;
        pose.rawY = rawY;

        if (id == 20) {
            pose.posX = rawX + CONSTX;
            pose.posY = fieldLength - rawY - CONSTY;
        } else if (id == 24) {
            pose.posX = fieldLength - rawX - CONSTX;
            pose.posY = fieldLength - rawY - CONSTY;
        }
    }
    */
}