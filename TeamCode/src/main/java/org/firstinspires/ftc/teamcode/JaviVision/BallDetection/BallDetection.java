package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;

import java.util.ArrayList;

public class BallDetection {

    public final LimelightPose pose = new LimelightPose();
    private final Limelight3A limelight;

    public BallDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    // ------------------------------------------------------------
    public double[] update() {
        double[] empty = {1};
        double[] inputs = {1, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,};
        limelight.updatePythonInputs(inputs);

        // ================= LIMELIGHT =================
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            pose.valid = false;
            return empty;
        }
        return result.getPythonOutput();
    }
}
