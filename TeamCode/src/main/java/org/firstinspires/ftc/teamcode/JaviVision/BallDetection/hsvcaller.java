package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous()
@Config
public class hsvcaller extends OpMode {
    BallDetection ll;
    private Telemetry dash;
    @Override
    public void init()
    {
        ll = new BallDetection(hardwareMap, 4);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        double[] results = ll.updateHSV();
        telemetry.addData("Hmin", results[0]);
        telemetry.addData("Hmax", results[3]);
        telemetry.addData("Smin", results[1]);
        telemetry.addData("Smax", results[4]);
        telemetry.addData("Vmin", results[2]);
        telemetry.addData("Vmax", results[5]);
        telemetry.addData("wait time", results[6]);
        telemetry.update();
        //telemetry.update();
        dash.update();
    }
}
