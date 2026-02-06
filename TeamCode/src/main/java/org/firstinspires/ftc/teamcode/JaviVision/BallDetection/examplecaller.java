package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

@Autonomous()
@Config
public class examplecaller extends OpMode {
    BallDetection ll;
    @Override
    public void init()
    {
        ll = new BallDetection(hardwareMap);
    }
    public void loop() {// <-- This refreshes pose
        telemetry.addData("output", ll.update()[0]);
    }
}
