package org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

@Autonomous()
@Config
@Disabled
public class examplecaller extends OpMode {
    LimelightProcessor_v3 ll;
    @Override
    public void init()
    {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        ll = new LimelightProcessor_v3(hardwareMap, odo);
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
            telemetry.addData("posX", 39.3701*posX);
            telemetry.addData("posY", 39.3701*posY);
            telemetry.addData("theta", ll.pose.theta);
            telemetry.addData("heading", ll.pose.heading);
            telemetry.addData("tx", ll.pose.tx);
            telemetry.addData("cornerX", 39.3701*ll.pose.cornerX);
            telemetry.addData("cornerY", 39.3701*ll.pose.cornerY);
            telemetry.addData("cos_value", ll.pose.cos_value);
            telemetry.addData("sin_value", ll.pose.sin_value);
            telemetry.addData("id",id);
        }
        else{
            telemetry.addData("valid", 0);
        }
    }
}
