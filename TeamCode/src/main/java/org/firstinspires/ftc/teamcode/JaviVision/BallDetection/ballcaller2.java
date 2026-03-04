package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Autonomous()
@Config
public class ballcaller2 extends OpMode {
    LimelightV5 ll;
    private Telemetry dash;
    @Override
    public void init()
    {
        ll = new LimelightV5(hardwareMap, 2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        ArrayList<Object[]> detections = ll.updateBall2();
        for (Object[] row : detections) {
            telemetry.addLine("----- New ball -----");
            telemetry.addData("CamX", row[0]);
            telemetry.addData("Distance", row[1]);
            telemetry.addData("Class ID", row[2]);
            telemetry.addData("Class Name", row[3]);
            telemetry.addData("Confidence", row[4]);
        }
        telemetry.update();
        dash.update();
    }
}
