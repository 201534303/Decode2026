package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightProcessor_v3 {

    public final LimelightPose pose = new LimelightPose();
    private Limelight3A limelight;
    private final double center = 3.6195/2; // x and y (IT'S A SQUARE DINGUS)

    private final double REDX = center - 0.381;
    private final double REDY = center - 0.314325;
    private final double BLUEX = 0.381 - center;
    private final double BLUEY = center - 0.314325;

    private final double DIAG = 0.371475;
    private final double DIAG2 = 0.3302;
    // field length = 3.6195


    public LimelightProcessor_v3(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && fiducials.size() > 0) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int id = fiducial.getFiducialId();

                Pose3D camPose = fiducial.getCameraPoseTargetSpace();
                Position position = camPose.getPosition();
                YawPitchRollAngles rotation = camPose.getOrientation();

                // In the current FTC Limelight SDK, you access fields directly:
                double camX = 39.37*position.x;
                double camY = 39.37*position.y;
                double camZ = 39.37*position.z;
                double camRoll = rotation.getRoll();
                double camPitch = rotation.getPitch();
                double camYaw = rotation.getYaw();
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
        double posX;
        double posY;

        // IF STATEMENT FOR RED
        if (pose.id == 24) {
            double redX = (DIAG2 - pose.x)*0.71933980033;
            double redY = (DIAG + (-pose.z))*0.80901699437 + pose.x*0.71933980033;
            posX = 3.6195 - redX;
            posY = 3.6195 - redY;
            pose.posX = posX;
            pose.posY = posY;
            pose.x = 0;
            pose.y = 0;
            pose.z = 0;
        }
        // IF STATEMENT FOR BL
        else if (pose.id == 20) {
            posX = 0;
            posY = 0;
        }
    }
}
