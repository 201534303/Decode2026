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

    private double stored_yaw;
    private double stored_shooter;
    private double stored_tx;
    private final double field = 3.606798;
    private final double halfPi = Math.PI/2;


    public LimelightProcessor_v3Tele(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }


    public void updateTele(double yawIn, double shooterIn) {
        //odo.update();
        //Pose2D pos = odo.getPosition();
        double yaw = yawIn;
        double shooterAngle = shooterIn;

        stored_yaw = yaw;
        stored_shooter = shooterAngle;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && fiducials.size() > 0) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int id = fiducial.getFiducialId();

                stored_tx = Math.toRadians(fiducial.getTargetXDegrees());

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
        double angle = Math.abs(stored_yaw) + stored_shooter;
        angle = Math.abs(angle);
        // IF STATEMENT FOR RED
        if (pose.id == 24) {
            double theta = Math.abs(Math.PI - angle - stored_tx);
            pose.heading = angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            double a = Math.abs(Math.cos(theta)) * pose.distance;
            double b = Math.abs(Math.sin(theta)) * pose.distance;
            posX = field - (CONSTX + a);
            posY = field - (CONSTY + b);
            pose.posX = posX;
            pose.posY = posY;
        }
        // IF STATEMENT FOR BL
        else if (pose.id == 20) {
            double theta = angle + stored_tx;
            pose.heading = angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            double a = Math.abs(Math.cos(theta)) * pose.distance;
            double b = Math.abs(Math.sin(theta)) * pose.distance;
            posX = CONSTX + a;
            posY = field - (CONSTY + b);
            pose.posX = posX;
            pose.posY = posY;
        }

        if (stored_yaw > halfPi) { stored_yaw -= halfPi; }
        transformation_angle = Math.abs(halfPi - stored_yaw);

        double cos_value = Math.abs(Math.cos(transformation_angle));
        double sin_value = Math.abs(Math.sin(transformation_angle));

        pose.cos_value = cos_value;
        pose.sin_value = sin_value;

        double dx = cos_value*(-0.1777999) - sin_value*(-0.20319989);
        double dy = sin_value*(-0.1777999) + cos_value*(-0.20319989);

        double cornerX = posX + dx;
        double cornerY = posY + dy;

        cos_value = Math.abs(Math.cos(transformation_angle));
        sin_value = Math.abs(Math.sin(transformation_angle));

        dx = sin_value*(2/39.3701);
        dy = cos_value*(-2/39.3701);

        double centerX = posX + dx;
        double centerY = posY + dy;


        pose.z = transformation_angle;
        transformation_angle += stored_shooter;
        pose.pitch = transformation_angle;

        cos_value = Math.abs(Math.cos(transformation_angle));
        sin_value = Math.abs(Math.sin(transformation_angle));

        dx = sin_value*(2/39.3701);
        dy = cos_value*(-2/39.3701);

        centerX = posX + dx;
        centerY = posY + dy;
        pose.roll = stored_shooter;
        pose.posX2 = centerX;
        pose.posY2 = centerY;

        pose.cornerX = cornerX;
        pose.cornerY = cornerY;
    }
}