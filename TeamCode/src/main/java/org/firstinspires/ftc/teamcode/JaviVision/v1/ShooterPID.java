package org.firstinspires.ftc.teamcode.JaviVision.v1;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.JaviVision.v1.LimelightProcessor;

@Autonomous()
public class ShooterPID extends LinearOpMode {

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
        LimelightProcessor ll = new LimelightProcessor(limelight);

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

            ll.update();   // <-- This refreshes pose
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            double yaw   = angles.getYaw(AngleUnit.DEGREES);    // heading
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll  = angles.getRoll(AngleUnit.DEGREES);


            telemetry.addData("Heading", yaw);
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Roll", roll);

            if (ll.pose.valid) {
                lastHeading = yaw;
                double id = ll.pose.id;

                // --- IMPORTANT: turret turning should use TX, NOT RX ---
                // RX is roll, not horizontal aim

                double tx = ll.pose.tx;   // left/right offset in inches

                // Basic proportional control
                power = tx * 0.007;  // 0.01–0.02 is typical for CR servos

                // Clamp power so servo does not go crazy
                power = Math.max(-0.5, Math.min(0.5, power));

                servo1.setPower(power);
                servo2.setPower(power);

                // Read IMU orientatio


                telemetry.addData("Target X Offset", tx);
                telemetry.addData("Turret Power", power);
                telemetry.addData("id", id);
            }
            else {
                if (lastHeading != 0) {
                    double diff = yaw - lastHeading;
                    power = diff * 0.007;
                    power = Math.max(-0.5, Math.min(0.5, power));
                    servo1.setPower(power);
                    servo2.setPower(power);
                    telemetry.addData("power",power);
                    telemetry.addData("last", lastHeading);
                }
            }

            telemetry.addData("Distance", ll.pose.distance);
            telemetry.update();
        }
    }
}
