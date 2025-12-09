package org.firstinspires.ftc.teamcode.JaviVision.v3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.v3.LimelightProcessor_v3;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous()
@Config
public class examplecaller extends OpMode {
    LimelightProcessor_v3 ll;
    @Override
    public void init()
    {
        ll = new LimelightProcessor_v3(hardwareMap);
    }
    public void loop() {
        ll.update();   // <-- This refreshes pose
        if (ll.pose.valid) {
            double id = ll.pose.id;
            double x = ll.pose.x;
            double y = ll.pose.y;
            double z = ll.pose.z;
            double yaw = ll.pose.yaw;
            double pitch = ll.pose.pitch;
            double roll = ll.pose.roll;
            double distance = ll.pose.distance;
            ll.getRobotPose();
            double posX = ll.pose.posX;
            double posY = ll.pose.posY;
            telemetry.addData("distance",39.3701*distance);
            telemetry.addData("yaw", yaw);
            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);
            telemetry.addData("posX", 39.3701*posX);
            telemetry.addData("posY", 39.3701*posY);
            telemetry.addData("x", 39.3701*x);
            telemetry.addData("y", 39.3701*y);
            telemetry.addData("z",39.3701*z);
            telemetry.addData("id",id);
        }
        else{
            telemetry.addData("valid", 0);
        }
    }
}
