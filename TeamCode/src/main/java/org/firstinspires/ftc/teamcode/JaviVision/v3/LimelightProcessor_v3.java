package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class LimelightProcessor_v3 extends LinearOpMode {

    Limelight3A limelight;

    double ROBOT_X = 0;
    double ROBOT_Y = 0;
    double RED_APRIL_TAG_X = 0.36863; // meters
    double APRIL_TAG_Y = 3.343275;
    double ANGLE = 37;
    // angles are wrong
    double BLUE_APRIL_TAG_X = 3.2893;
    //double BLUE_APRIL_TAG_ANGLE = -124

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Ensure we’re using your AprilTag pipeline
        limelight.pipelineSwitch(5);
        limelight.start();

        telemetry.addLine("Limelight initialized — waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
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
                    double camX = position.x;
                    double camY = position.y;
                    double camZ = position.z;
                    double camRoll = rotation.getRoll();
                    double camPitch = rotation.getPitch();
                    double camYaw = rotation.getYaw();

                    double distance = Math.sqrt(camX*camX+camZ*camZ);
                    /*double down_theta = Math.toRadians(camYaw + ANGLE - 90);
                    double down = Math.sin(down_theta)*distance;
                    double right_theta = Math.toRadians(180-camYaw-37);
                    double right = Math.sin(right_theta)*distance;

                    ROBOT_Y = APRIL_TAG_Y - position.y;

                    if (id == 24) {
                        ROBOT_X = RED_APRIL_TAG_X + position.x;
                    }
                    else if (id == 20) {
                        ROBOT_X = BLUE_APRIL_TAG_X - position.x;
                    }
                    */

                    telemetry.addData("roll", camRoll);
                    telemetry.addData("pitch", camPitch);
                    telemetry.addData("yaw", camYaw);
                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Cam X (m)", "%.3f", camX);
                    telemetry.addData("Cam Y (m)", "%.3f", camY);
                    telemetry.addData("Cam Z (m)", "%.3f", camZ);
                    //telemetry.addData("Pipeline", limelight.getActivePipelineIndex());
                } else {
                    telemetry.addLine("No fiducials detected.");
                }
            } else {
                telemetry.addLine("No valid Limelight result yet.");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
