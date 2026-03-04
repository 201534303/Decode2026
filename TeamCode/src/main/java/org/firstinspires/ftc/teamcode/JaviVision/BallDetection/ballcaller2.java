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
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = (nowTime - lastTime);
        lastTime = nowTime;

        ArrayList<Object[]> detections = ll.updateBall2();
        for (Object[] row : detections) {
            String className = row[3].toString();
            int id = (int) row[2];
            double camX = (double) row[0];
            double camY = (double) row[1];
            Double[] ret = {camX, camY};
            if (className.equals("purple")) {
                if (id >= purpleBalls.size()) {
                    purpleBalls.add(ret);
                }
                else {
                    double oldX = purpleBalls.get(id)[0];
                    double oldY = purpleBalls.get(id)[1];
                    purpleBalls.set(id, ret);
                    double velX = (camX-oldX)/timeDif;
                    double velY = (camY-oldY)/timeDif;
                    telemetry.addData("VelX", velX);
                    telemetry.addData("VelY", velY);
                }
            }
            telemetry.addLine("----- New ball -----");
            telemetry.addData("CamX", row[0]);
            telemetry.addData("CamY", row[1]);
            telemetry.addData("Class ID", row[2]);
            telemetry.addData("Class Name", row[3]);
            telemetry.addData("Confidence", row[4]);
            telemetry.addData("Distance", row[5]);
        }
        telemetry.update();
        dash.update();
    }
}
