package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous()
@Config
public class ballcaller2 extends OpMode {
    private ElapsedTime overallRuntime;
    private double lastTime;
    private double timeDif;
    LimelightV5 ll;
    private Telemetry dash;
    private ArrayList<Double[]> purpleBalls = new ArrayList<>();
    private ArrayList<Double[]> greenBalls = new ArrayList<>();
    @Override
    public void init()
    {
        ll = new LimelightV5(hardwareMap, 2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        ArrayList<double[]> detections = ll.updateBall2();
        for (double[] ball : detections) {
            telemetry.addLine(" ---- BALL ----");
            telemetry.addData("Ball x", ball[0]);
            telemetry.addData("Ball y", ball[1]);
            telemetry.addData("Vel x", ball[2]);
            telemetry.addData("Vel y", ball[3]);
        }

        telemetry.update();
        dash.update();
    }
}
