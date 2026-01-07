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

    private double stored_yaw;
    private double stored_shooter;
    private double stored_tx;
    private final double fieldLength = 144;
    private final double halfPi = Math.PI/2;

    private final double alpha = 0.25;


    public LimelightProcessor_v3Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }


    public void updateTele() {
        //odo.update();
        //Pose2D pos = odo.getPosition();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && fiducials.size() > 0) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int id = fiducial.getFiducialId();

                stored_tx = Math.toRadians(fiducial.getTargetXDegrees());
                pose.tx = stored_tx;

                Pose3D camPose = fiducial.getCameraPoseTargetSpace();
                Position position = camPose.getPosition();
                YawPitchRollAngles rotation = camPose.getOrientation();

                // In the current FTC Limelight SDK, you access fields directly:
                double camX = position.x;
                double camY = position.y;
                double camZ = position.z;
                double camRoll = rotation.getRoll();
                double camPitch = rotation.getPitch();
                double distance = Math.sqrt(camX*camX+camZ*camZ) * 39.3701;
                if (camY < 0) {
                    pose.x = -camX;
                    pose.y = -camY;
                }
                else {
                    pose.x = camX;
                    pose.y = camY;
                }
                pose.z = camZ;
                pose.pitch = camPitch;
                pose.roll = camRoll;
                pose.distance = distance;
                pose.id = id;
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

    public void getRobotPose(double yawIn, double shooterIn, double delAngle, int id) {
        double yaw = yawIn;
        // CHECK FOR BLUE
        if (yaw > 90) {
            yaw = 180 - yaw;
        }
        /*if (stored_tx < 0) {
            stored_tx += Math.toRadians(2);
        }
        else if (stored_tx > 0) {
            stored_tx -= Math.toRadians(2);
        }*/
        double theta = Math.abs(yaw + Math.toRadians(shooterIn) - 0.8*stored_tx);
        pose.theta = theta;
        pose.heading = yaw;
        double rawX = (6.5 + pose.distance)*Math.cos(theta);
        double rawY = (6.5 + pose.distance)*Math.sin(theta);
        pose.rawX = rawX;
        pose.rawY = rawY;
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
}