package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous()
@Config
public class ballcaller extends OpMode {
    BallDetection ll;
    private Telemetry dash;
    @Override
    public void init()
    {
        ll = new BallDetection(hardwareMap, 0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        double[] results = ll.updateBall();
        double count = 0;
        for (double result : results) {
            if (count%2 ==0) {
                telemetry.addData("X", result);
            }
            else {
                telemetry.addData("Y", result);
            }
            count += 1;
        }
        telemetry.update();
        dash.update();
    }
}
