package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

@Autonomous()
@Config
public class examplecaller extends OpMode {
    BallDetection ll;
    private Telemetry dash;
    @Override
    public void init()
    {
        ll = new BallDetection(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        // if you change the update functino type you have to change pipeline too!
        double[] results = ll.updateBall();
        for (double result: results) {
            if (result != 0) {
                if (result%2==0) {
                    telemetry.addData("X", result);
                }
                else {
                    telemetry.addData("Y", result);
                }
            }
        }
        telemetry.update();
        dash.update();
    }
}
