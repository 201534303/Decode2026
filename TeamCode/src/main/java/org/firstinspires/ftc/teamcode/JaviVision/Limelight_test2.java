/*package org.firstinspires.ftc.teamcode.JaviVision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.Limelight_Test;

@Autonomous()
public class Limelight_test2 extends LinearOpMode {

    public CRServo servo1;
    public CRServo servo2;
    double power;
    double lastHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        // Processor that extracts tx/ty/tz/rx/ry/rz/distance
        Limelight_Test ll = new Limelight_Test(hardwareMap);

        // Servos
        servo1 = hardwareMap.get(CRServo.class, "turret_servo1");
        servo2 = hardwareMap.get(CRServo.class, "turret_servo2");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            double yaw   = angles.getYaw(AngleUnit.DEGREES);    // heading
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll  = angles.getRoll(AngleUnit.DEGREES);


            telemetry.addData("Heading", yaw);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Roll", roll);

            double distance = ll.distanceFromTag(24); 

            telemetry.addData("distance",distance);
        }
    }
}
*/