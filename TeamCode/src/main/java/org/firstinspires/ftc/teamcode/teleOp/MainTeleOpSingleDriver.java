package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.v3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;

@TeleOp(name="Single Driver Tele", group="Iterative OpMode")
@Config
public class MainTeleOpSingleDriver extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private IntakeTele intake;
    private Follower follower;
    double aimPower;

    ShooterTele shooter;
    private Telemetry dash;
    public static double power = 0;
    public static double kf = 0;
    public static double kp1 = 0.007;

    public static double ki1 = 0.0001;

    public static double kd1 = 0.004;
    public static double kp2 = 0.007;
    public static double ki2 = 0.0001;
    public static double kd2 = 0.004;
    private double lastTurretRotation = 0;
    private double lastPinpoint = 0;
    private double lastTX = 100;

    public double cameraLoopCounter = 0;

    LimelightProcessor_v3Tele ll;

    public final Pose homing = new Pose(72, 0, -Math.toRadians(90));
    public Pose reset = new Pose(0, 0, 0);



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
        intake = new IntakeTele(hardwareMap, gamepad1, gamepad2, telemetry);
        shooter = new ShooterTele(hardwareMap, gamepad1, gamepad2, telemetry, runtime);
        telemetry.addData("Status", "Initialized");
        ll = new LimelightProcessor_v3Tele(hardwareMap);



        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //driving
        if (gamepad1.share){
            follower.setPose(homing);
        }
        reset = new Pose (follower.getPose().getX(), follower.getPose().getY(), 0);
        if (gamepad1.options){
            follower.setPose(reset);
        }
        follower.update();

        ll.updateTele(follower.getPose().getHeading(), shooter.getTTPos());
        ll.getRobotPose();
        telemetry.addData("distance", ll.pose.distance * 39.3701);
        telemetry.addData("cornerX", ll.pose.cornerX * 39.3701);
        telemetry.addData("cornerY", ll.pose.cornerY * 39.3701);
        telemetry.addData("posX", ll.pose.posX * 39.3701);
        telemetry.addData("posY", ll.pose.posY * 39.3701);
        telemetry.addData("centerX", ll.pose.posX2 * 39.3701);
        telemetry.addData("centerY", ll.pose.posY2 * 39.3701);
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("stored_shooter",ll.pose.roll);
        telemetry.addData("trans_angle", Math.toDegrees(ll.pose.pitch));
        telemetry.addData("trans_angle2", Math.toDegrees(ll.pose.z));
        dt.feildCentricDrive(Math.toDegrees(follower.getPose().getHeading()));
        //dt.updateOdo();

        if (((ll.pose.id == 20) || (ll.pose.id == 24)) && ll.pose.valid) {
            aimPower = shooter.PIDF(ll.pose.tx, 0, kp1, ki1, kd1, kf);
            aimPower = Math.max(-0.5, Math.min(0.5, aimPower));

            if (Math.abs(ll.pose.tx) < lastTX) {
                lastTX = ll.pose.tx;
                lastPinpoint = follower.getHeading();
                lastTurretRotation = shooter.getTTPos();
            }
        }
        else {
            if (lastTX != 100) {
                double turret_error = Math.toDegrees(shooter.getTTPos()) - Math.toDegrees(lastTurretRotation);
                //double turret_error = 0;
                double heading_error = Math.toDegrees(Math.abs(follower.getPose().getHeading())) - Math.toDegrees(Math.abs(lastPinpoint));
                //double heading_error = 0;
                double angle = turret_error - heading_error;
                /*
                telemetry.addLine("//////");
                telemetry.addData("angle", angle);
                telemetry.addData("current turret", Math.toDegrees(shooter.getTTPos()));
                telemetry.addData("last turret", Math.toDegrees(lastTurretRotation));
                telemetry.addData("current heading", Math.toDegrees(Math.abs(follower.getPose().getHeading())));
                telemetry.addData("last heading", Math.toDegrees(Math.abs(lastPinpoint)));
                telemetry.addLine("////");*/

                aimPower = shooter.PIDF(angle, 0, kp2, ki2, kd2, kf);
                if (aimPower > 0) {
                    aimPower = Math.max(0.05, Math.min(0.5, aimPower));
                }
                else {
                    aimPower = Math.max(-0.5, Math.min(-0.05, aimPower));
                }
            }
        }
        telemetry.addData("power", -aimPower);
        shooter.setTurretPower(0);

        //intake
        intake.updateSingle();
        //shooter
        shooter.shooterMachineSingle();
        
        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("tt pos", Math.toDegrees(shooter.getTTPos()));

        telemetry.addData("tx", Math.toDegrees(ll.pose.tx));
        //telemetry.addData("cornerX", 39.3701*ll.pose.cornerX);
        //telemetry.addData("cornerY", 39.3701*ll.pose.cornerY);

        //dash.addData("shooter vel", shooter.getMotorVel());
        //dash.addData("target vel", kf);
        dash.addData("flywheel rpm", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
        cameraLoopCounter += 1;
    }

    @Override
    public void stop() {
    }
}
