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
<<<<<<< Updated upstream
        double[] inputs = {2, 0, 0, 0, 255, 255, 255, 0};
=======
        double[] purple = {173, 109, 0, 8, 255, 107};
        double purpleH = 100000 + purple[0]*1000 + purple[3];
        double purpleS = 100000 + purple[1]*1000 + purple[4];
        double purpleV = 100000 + purple[2]*1000 + purple[5];
        double[] green = {};
        double[] inputs = {2, purpleH, purpleS, purpleV, 1, 0, 0, 0};
        //                 P, Hmin, Smin, Vmin, Hmax, Smax, vmax, 0
>>>>>>> Stashed changes
        limelight.updatePythonInputs(inputs);
        LLResult result = limelight.getLatestResult();
        return result.getPythonOutput();
    }
}
