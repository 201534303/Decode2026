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
    private final double center = 3.6195/2; // x and y (IT'S A SQUARE DINGUS)

    private final double CONSTX = 0.4572;
    private final double CONSTY = 0.314325;

    private final double DIAG = 0.371475;
    private final double DIAG2 = 0.3302;

    private double stored_angle;
    private double stored_tx;
    private final double field = 3.606798;


    public LimelightProcessor_v3Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }


    public void updateTele(double yawIn) {
        //odo.update();
        //Pose2D pos = odo.getPosition();
        double yaw = yawIn;

        stored_angle = yaw;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && fiducials.size() > 0) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int id = fiducial.getFiducialId();

                stored_tx = fiducial.getTargetXDegrees();

                Pose3D camPose = fiducial.getCameraPoseTargetSpace();
                Position position = camPose.getPosition();
                YawPitchRollAngles rotation = camPose.getOrientation();

                // In the current FTC Limelight SDK, you access fields directly:
                double camX = position.x;
                double camY = position.y;
                double camZ = position.z;
                double camRoll = rotation.getRoll();
                double camPitch = rotation.getPitch();
                double camYaw = yaw;
                double distance = Math.sqrt(camX*camX+camZ*camZ);
                if (camY < 0) {
                    pose.x = -camX;
                    pose.y = -camY;
                }
                else {
                    pose.x = camX;
                    pose.y = camY;
                }
                pose.z = camZ;
                pose.yaw = camYaw;
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

    public void getRobotPose() {
        double posX = 0;
        double posY = 0;
        double transformation_angle = 0;
        // IF STATEMENT FOR RED
        if (pose.id == 24) {
            double theta = 180 - Math.abs(stored_angle) - stored_tx;
            pose.heading = stored_angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            transformation_angle = Math.abs(Math.abs(stored_angle) - 90);
            double a = Math.abs(Math.cos(Math.toRadians(theta))) * pose.distance;
            double b = Math.abs(Math.sin(Math.toRadians(theta))) * pose.distance;
            posX = field - (CONSTX + a);
            posY = field - (CONSTY + b);
        }
        // IF STATEMENT FOR BL
        else if (pose.id == 20) {
            double theta = Math.abs(stored_angle) + stored_tx;
            pose.heading = stored_angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            transformation_angle = Math.abs(90 - Math.abs(stored_angle));
            double a = Math.abs(Math.cos(Math.toRadians(theta))) * pose.distance;
            double b = Math.abs(Math.sin(Math.toRadians(theta))) * pose.distance;
            posX = CONSTX + a;
            posY = field - (CONSTY + b);
        }
        pose.posX = posX;
        pose.posY = posY;

        double cos_value = Math.abs(Math.cos(Math.toRadians(transformation_angle)));
        double sin_value = Math.abs(Math.sin(Math.toRadians(transformation_angle)));

        pose.cos_value = cos_value;
        pose.sin_value = sin_value;
        
        double dx = cos_value*(-0.1777999) - sin_value*(-0.20319989);
        double dy = sin_value*(-0.1777999) + cos_value*(-0.20319989);

        double cornerX = posX + dx;
        double cornerY = posY + dy;

        pose.cornerX = cornerX;
        pose.cornerY = cornerY;
    }
}