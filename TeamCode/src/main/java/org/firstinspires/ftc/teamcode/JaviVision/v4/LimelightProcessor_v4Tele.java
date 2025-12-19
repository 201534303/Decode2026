package org.firstinspires.ftc.teamcode.JaviVision.v4;

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


public class LimelightProcessor_v4Tele {

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


    public LimelightProcessor_v4Tele(HardwareMap hardwareMap) {
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

        if (!pose.valid) return;

        // =========================
        // 1. Tag position (field)
        // =========================
        double tagX, tagY;

        if (pose.id == 24) {       // RED
            tagX = field - CONSTX;
            tagY = field - CONSTY;
        } else if (pose.id == 20) { // BLUE
            tagX = CONSTX;
            tagY = field - CONSTY;
        } else {
            pose.valid = false;
            return;
        }

        // =========================
        // 2. Camera → robot center offset
        // Rotate into field frame using robot heading
        // =========================
        double robotHeading = Math.abs(stored_yaw); // IMU: -90° = facing +Y
        double offsetX = -1.5/39.3701 * Math.cos(robotHeading);
        double offsetY = -1.5/39.3701 * Math.sin(robotHeading);

        double offsetMag = Math.hypot(offsetX, offsetY); // camera → center distance

        // =========================
        // 3. Turret angle in standard unit circle
        // =========================
        double turretAngleStd = Math.PI/2 - stored_shooter; // convert your turret convention

        // =========================
        // 4. Law of cosines to get tag → robot center distance
        // =========================
        double tagToCenterDist = Math.sqrt(
                pose.distance * pose.distance
                        + offsetMag * offsetMag
                        - 2 * pose.distance * offsetMag * Math.cos(turretAngleStd)
        );

        // =========================
        // 5. Law of sines to get angle opposite the camera offset
        // (triangle: tag-camera-center)
        // =========================
        double theta = robotHeading + stored_tx;

        // =========================
        // 6. Decompose into X/Y components relative to field
        // =========================
        double dx = tagToCenterDist * Math.cos(robotHeading + theta);
        double dy = tagToCenterDist * Math.sin(robotHeading + theta);

        // =========================
        // 7. Compute robot center position
        // =========================
        pose.posX = tagX - dx;
        pose.posY = tagY - dy;

        // =========================
        // 8. Optional: store additional info
        // =========================
        pose.heading = robotHeading;  // field heading
        pose.theta = theta;      // triangle angle for debug / visualization
        pose.posX2 = pose.posX;       // can apply further transforms if needed
        pose.posY2 = pose.posY;
    }


}