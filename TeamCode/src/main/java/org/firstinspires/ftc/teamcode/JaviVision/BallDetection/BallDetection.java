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
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ------------------------------------------------------------
    public double[] update() {
        // ================= LIMELIGHT =================
        double[] inputs = {2, 0, 0, 0, 255, 255, 255, 0};
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
}
