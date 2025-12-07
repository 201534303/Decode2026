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
                pose.x = camX;
                pose.y = camY;
                pose.z = camZ;
                pose.yaw = camYaw;
                pose.pitch = camPitch;
                pose.roll = camRoll;
                pose.distance = distance;
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

        // IF STATEMENT FOR BLUE
        if (pose.id == 20) {
            posX = 0;
            posY = 0;
        }
        // IF STATEMENT FOR RED
        else if (pose.id == 24) {
            posX = 0;
            posY = 0;
        }
        return 0;
    }
}
