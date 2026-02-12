package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.JaviVision.Position.Pose.LimelightPose;

import java.util.ArrayList;

public class BallDetection {

    public final LimelightPose pose = new LimelightPose();
    private final Limelight3A limelight;

    public BallDetection(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // update Ball pipeline
        //limelight.pipelineSwitch(4); // HSVTuner pipeline
        limelight.start();
    }

    // ------------------------------------------------------------
    public double[] updateBall() {
        // ================= LIMELIGHT =================
        double[] purple = {153, 30, 0, 13, 255, 80};
        double purpleH = 100000 + purple[0]*1000 + purple[3];
        double purpleS = 100000 + purple[1]*1000 + purple[4];
        double purpleV = 100000 + purple[2]*1000 + purple[5];
        double[] green = {0,0,0,0,0,0};
        double[] inputs = {1, purpleH, purpleS, purpleV, 0, 0, 0, 0};
        //             P, Hmin, Smin, Vmin, Hmax, Smax, vmax, 0
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
    public double[] updateHSV() {
        // ================= LIMELIGHT =================
        double[] inputs = {1, 0, 0, 0, 0, 0, 0, 0};
        //                 P, Hmin, Smin, Vmin, Hmax, Smax, vmax, 0
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
}
