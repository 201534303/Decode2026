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
    private double timeDif = 0.0;
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
        overallRuntime = new ElapsedTime();
    }
    public void loop() {// <-- This refreshes pose
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = (nowTime - lastTime);
        lastTime = nowTime;

        //ArrayList<double[]> detections = ll.updateBall2(timeDif, );
// Before the detection loop
        //telemetry.addData("Total detections", detections.size());

// Inside the loop, differentiate by id
        /*for (double[] ball : detections) {
            telemetry.addLine("---- new Ball ----");
            telemetry.addData("x ", ball[0]);
            telemetry.addData("y ", ball[1]);*/
            /*
                // normal display code
            if (ball[4] > 0) { // id > 0 means it's a tracked ball
                telemetry.addLine("--- TRACKED ---");
                telemetry.addData("id",    ball[4]);
                telemetry.addData("vx",    ball[2]);
                telemetry.addData("vy",    ball[3]);
            } else {
                telemetry.addLine("--- RAW ---");
                telemetry.addData("camX",  ball[0]);

                telemetry.addData("dist",  ball[1]);
             }
             */
        }

       // telemetry.update();
        //dash.update();
    }
