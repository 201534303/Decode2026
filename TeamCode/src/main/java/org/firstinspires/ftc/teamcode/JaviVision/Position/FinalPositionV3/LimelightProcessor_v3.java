package org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class LimelightProcessor_v3 {

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
    private final double halfPi = Math.PI/2;


    public LimelightProcessor_v3(HardwareMap hardwareMap, GoBildaPinpointDriver odo) {
        this.odo = odo;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    public void update() {
        odo.update();
        Pose2D pos = odo.getPosition();
        double yaw = pos.getHeading(AngleUnit.DEGREES);

        stored_angle = yaw;

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
        stored_angle = Math.abs(stored_angle);
        // IF STATEMENT FOR RED
        if (pose.id == 24) {
            double theta = Math.abs(Math.PI - stored_angle - 1.12*stored_tx);
            pose.heading = stored_angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            double a = Math.abs(Math.cos(theta)) * pose.distance;
            double b = Math.abs(Math.sin(theta)) * pose.distance;
            pose.x = a;
            pose.y = b;
            pose.posX2 = a;
            pose.posY2 = b;
            posX = field - (CONSTX + a);
            posY = field - (CONSTY + b);
        }
        // IF STATEMENT FOR BL
        else if (pose.id == 20) {
            double theta = stored_angle + 1.12*stored_tx;
            pose.heading = stored_angle;
            pose.tx = stored_tx;
            pose.theta = theta;
            double a = Math.abs(Math.cos(theta)) * pose.distance;
            double b = Math.abs(Math.sin(theta)) * pose.distance;
            pose.x = a;
            pose.y = b;
            pose.posX2 = a;
            pose.posY2 = b;
            posX = CONSTX + a;
            posY = field - (CONSTY + b);
        }
        if (stored_angle > halfPi) { stored_angle -= halfPi; }
        transformation_angle = Math.abs(halfPi - stored_angle);

        double cos_value = Math.abs(Math.cos(transformation_angle));
        double sin_value = Math.abs(Math.sin(transformation_angle));

        pose.cos_value = cos_value;
        pose.sin_value = sin_value;

        double dx = cos_value*(-0.1777999) - sin_value*(-0.20319989);
        double dy = sin_value*(-0.1777999) + cos_value*(-0.20319989);

        double cornerX = posX + dx;
        double cornerY = posY + dy;
        dx = sin_value*(2/39.3701);
        dy = cos_value*(-2/39.3701);

        double centerX = posX + dx;
        double centerY = posY + dy;
        pose.posX = centerX;
        pose.posY = centerY;

        pose.cornerX = cornerX;
        pose.cornerY = cornerY;
    }
}