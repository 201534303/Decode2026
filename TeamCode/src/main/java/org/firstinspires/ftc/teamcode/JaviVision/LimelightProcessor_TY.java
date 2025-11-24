package org.firstinspires.ftc.teamcode.JaviVision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight 2D Distance", group = "Testing")
public class LimelightProcessor_TY extends LinearOpMode {

    private Limelight3A limelight;

    // ---- CHANGE THESE FOR YOUR ROBOT ----
    private static final double CAMERA_HEIGHT = 11.5;   // inches
    private static final double TARGET_HEIGHT = 29.4375;   // inches
    private static final double CAMERA_ANGLE = 0.0;    // degrees upward tilt
    // --------------------------------------

    private double last_tx = 0;
    private double last_ty = 0;
    private double last_ta = 0;
    private double last_distance = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (tags != null && tags.size() > 0) {

                    LLResultTypes.FiducialResult tag = tags.get(0); // take the first detected tag

                    double tx = tag.getTargetXDegrees();       // horizontal deg
                    double ty = tag.getTargetYDegrees();       // vertical deg
                    double ta = tag.getTargetArea();       // area

                    last_tx = tx;
                    last_ty = ty;
                    last_ta = ta;

                    // ---- DISTANCE CALC ----
                    double angleToTag = CAMERA_ANGLE + ty;
                    double angleRad = Math.toRadians(angleToTag);

                    double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
                    double distance = heightDiff / Math.tan(angleRad);

                    last_distance = distance;

                    telemetry.addLine("---- ll-info ----");
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("ta", ta);

                    telemetry.addLine("---- estimated ----");
                    telemetry.addData("distance (in)", "%.2f", distance);

                } else {
                    telemetry.addLine("No AprilTags detected.");
                }

            } else {
                telemetry.addLine("No LLResult available.");
                telemetry.addData("Last tx", last_tx);
                telemetry.addData("Last ty", last_ty);
                telemetry.addData("Last ta", last_ta);
                telemetry.addData("Last distance", last_distance);
            }

            telemetry.update();
            sleep(20);
        }
    }
}
