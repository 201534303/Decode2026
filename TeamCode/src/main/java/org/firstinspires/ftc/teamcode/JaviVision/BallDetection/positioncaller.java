package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous()
@Config
public class positioncaller extends OpMode {
    BallDetection ll;
    private Telemetry dash;
    @Override
    public void init()
    {
        ll = new BallDetection(hardwareMap, 3);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }
    public void loop() {// <-- This refreshes pose
        double[] results = ll.updatePos();
        if (results[0] == 0) {
            telemetry.addLine("I don't see nothing boss");
        }
        else if (results[0] == 1) {
            double x = results[1];
            double y = results[2];
            double z = results[3];
            double raw_yaw = results[4];
            double yaw = 36 -raw_yaw;
            double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z,2));
            telemetry.addData("X: ", x);
            telemetry.addData("Y:", y);
            telemetry.addData("Z", z);
            telemetry.addData("Raw Yaw: ", raw_yaw);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("X (cos):", distance*Math.cos(Math.toRadians(yaw)));
            telemetry.addData("Z (sin):", distance*Math.sin(Math.toRadians(yaw)));
        }
        telemetry.update();
        dash.update();
    }
}
