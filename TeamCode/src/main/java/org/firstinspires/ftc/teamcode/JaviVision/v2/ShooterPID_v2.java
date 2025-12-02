package org.firstinspires.ftc.teamcode.JaviVision.v2;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.v2.LimelightProcessor_v2;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

@Autonomous()
public class ShooterPID_v2 extends LinearOpMode {

    public CRServo servo1;
    public CRServo servo2;
    double power;
    double lastHeading = -10000;
    public GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();

        // Processor that extracts tx/ty/tz/rx/ry/rz/distance
        LimelightProcessor_v2 ll = new LimelightProcessor_v2(hardwareMap);

        // Servos
        servo1 = hardwareMap.get(CRServo.class, "turret_servo1");
        servo2 = hardwareMap.get(CRServo.class, "turret_servo2");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {

            ll.update();   // <-- This refreshes pose
            Pose2D pos = odo.getPosition();
            double yaw = pos.getHeading(AngleUnit.DEGREES);

            if (ll.pose.valid) {
                lastHeading = yaw;
                double id = ll.pose.id;

                // --- IMPORTANT: turret turning should use TX, NOT RX ---
                // RX is roll, not horizontal aim

                double tx = ll.pose.tx;   // left/right offset in inches

                // Basic proportional control
                power = tx * 0.01;  // 0.01–0.02 is typical for CR servos

                // Clamp power so servo does not go crazy
                power = Math.max(-1, Math.min(1, power));

                servo1.setPower(power);
                servo2.setPower(power);

                // Read IMU orientatio


                telemetry.addData("Target X Offset", tx);
                telemetry.addData("Turret Power", power);
                telemetry.addData("id", id);
            }
            else {
                if (lastHeading != -10000) {
                    double diff = yaw - lastHeading;
                    power = diff * 0.08;
                    power = Math.max(-1, Math.min(1, power));
                    servo1.setPower(power);
                    servo2.setPower(power);
                    telemetry.addData("power",power);
                    telemetry.addData("current yaw", yaw);
                    telemetry.addData("last headong", lastHeading);
                    telemetry.addData("diff", diff);
                }
            }

            telemetry.addData("Distance", ll.pose.distance);
            telemetry.update();
        }
    }
}
