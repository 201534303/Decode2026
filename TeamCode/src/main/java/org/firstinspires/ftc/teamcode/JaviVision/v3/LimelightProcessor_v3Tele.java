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

    private final double CONSTX = 17/39.3701;
    private final double CONSTY = 0.314325;

    private final double DIAG = 0.371475;
    private final double DIAG2 = 0.3302;

    private double stored_yaw;
    private double stored_shooter;
    private double stored_tx;
    private final double field = 144/39.3701;
    private final double halfPi = Math.PI/2;


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

                Pose3D camPose = fiducial.getCameraPoseTargetSpace();
                Position position = camPose.getPosition();
                YawPitchRollAngles rotation = camPose.getOrientation();

                // In the current FTC Limelight SDK, you access fields directly:
                double camX = position.x;
                double camY = position.y;
                double camZ = position.z;
                double camRoll = rotation.getRoll();
                double camPitch = rotation.getPitch();
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

    public void getRobotPose(double yawIn, double shooterIn) {
        double yaw = -yawIn + Math.PI/2;
        double theta = Math.abs(stored_shooter + yawIn + shooterIn);
        double rawX = (1.5/39.3701 + pose.distance)*Math.sin(theta);
        double rawY = (1.5/39.3701 + pose.distance)*Math.cos(theta);
        pose.posX = rawX;
        pose.posY = rawY;
    }
}