package org.firstinspires.ftc.teamcode.JaviVision.v2;

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
import org.firstinspires.ftc.teamcode.JaviVision.v2.LimelightProcessor_v2;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous()
@Config
public class ShooterPID_v2 extends OpMode {

    public CRServo servo1;
    public CRServo servo2;
    double power;
    double lastHeading = -10000;
    double oldTime;
    public static double kpTX = 0.02;
    public static double kiTX = 0.0001;
    public static double kdTX = 0.0001;
    public static double kpSearching = 0.001;
    public static double kiSearching = 0.0001;
    public static double kdSearching = 0.0001;
    double lastTX;
    public static double alpha = 0.8;
    double integralSumTX = 0;
    double lastDiff;
    double integralSumSearching;
    boolean detected = false;
    private Telemetry dash;
    LimelightProcessor_v2 ll;
    public GoBildaPinpointDriver odo;
    @Override
    public void init()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();

        // Initialize Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();

        // Processor that extracts tx/ty/tz/rx/ry/rz/distance
        ll = new LimelightProcessor_v2(hardwareMap);

        // Servos
        servo1 = hardwareMap.get(CRServo.class, "turret_servo1");
        servo2 = hardwareMap.get(CRServo.class, "turret_servo2");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
    }

    public void loop() {

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        oldTime = newTime;

        odo.update();
        ll.update();   // <-- This refreshes pose
        Pose2D pos = odo.getPosition();
        double yaw = pos.getHeading(AngleUnit.DEGREES);
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        if (ll.pose.valid) {
            detected = true;
            lastHeading = yaw;
            double id = ll.pose.id;

            double raw_tx = ll.pose.tx;
            
            double tx = alpha*lastTX + (1-alpha)*raw_tx;

            // rate of change of the error
            double derivative = (tx - lastTX) / loopTime;

            // sum of all error over time
            integralSumTX += (tx * loopTime);

            power = (kpTX * tx) + (kiTX * integralSumTX) + (kdTX * derivative);

            lastTX = tx;
            // Basic proportional control

            // Clamp power so servo does not go crazy
            power = Math.max(-1, Math.min(1, power));

            servo1.setPower(power);
            servo2.setPower(power);

            // Read IMU orientatio


            telemetry.addData("Target X Offset", tx);
            telemetry.addData("Turret Power", power);
            telemetry.addData("id", id);
            dash.addData("kp", kpTX*tx);
            dash.addData("ki", kiTX*integralSumSearching);
            dash.addData("kd", kdTX*derivative);
            dash.addData("Target X Offset", tx);
            dash.addData("Turret Power", power);
            dash.addData("id", id);
            telemetry.update();
            dash.update();
        }
        else {
            if (detected) {
                double diff = yaw - lastHeading;
                double derivative = (diff - lastDiff) / loopTime;

                integralSumSearching += (diff * loopTime);

                power = (kpSearching*diff) + (kiSearching * integralSumSearching) + (kdSearching * integralSumSearching);

                lastDiff = diff;

                power = Math.max(-1, Math.min(1, power));
                servo1.setPower(power);
                servo2.setPower(power);
                telemetry.addData("power",power);
                telemetry.addData("current yaw", yaw);
                telemetry.addData("last headong", lastHeading);
                telemetry.addData("diff", diff);
                dash.addData("kp", kpSearching*diff);
                dash.addData("ki", kiSearching*integralSumSearching);
                dash.addData("kd", kdSearching*derivative);
                dash.addData("power",power);
                dash.addData("current yaw", yaw);
                dash.addData("last headong", lastHeading);
                dash.addData("diff", diff);
                telemetry.update();
                dash.update();
            }
        }

        telemetry.addData("Distance", ll.pose.distance);
        telemetry.update();
    }
    /*
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();

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

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            oldTime = newTime;

            odo.update();
            ll.update();   // <-- This refreshes pose
            Pose2D pos = odo.getPosition();
            double yaw = pos.getHeading(AngleUnit.DEGREES);
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            if (ll.pose.valid) {
                detected = true;
                lastHeading = yaw;
                double id = ll.pose.id;

                double tx = ll.pose.tx;

                // rate of change of the error
                double derivative = (tx - lastTX) / loopTime;

                // sum of all error over time
                integralSumTX += (tx * loopTime);

                power = (kpTX * tx) + (kiTX * integralSumTX) + (kdTX * derivative);

                lastTX = tx;
                // Basic proportional control

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
                if (detected) {
                    double diff = yaw - lastHeading;
                    double derivative = (diff - lastDiff) / loopTime;

                    integralSumSearching += (diff * loopTime);

                    power = (kpSearching*diff) + (kiSearching * integralSumSearching) + (kdSearching * integralSumSearching);

                    lastDiff = diff;

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
     */
}
