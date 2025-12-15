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
    public static double kp = 0.02;

    public static double ki = 0;

    public static double kd = 0;
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

        ll.updateTele(Math.toDegrees(follower.getPose().getHeading()));
        ll.getRobotPose();
        dt.feildCentricDrive(Math.toDegrees(follower.getPose().getHeading()));
        //dt.updateOdo();

        if (((ll.pose.id == 20) || (ll.pose.id == 24)) && ll.pose.valid) {
            aimPower = shooter.PIDF(ll.pose.tx, 0, kp, ki, kd, kf);
            aimPower = Math.max(-0.5, Math.min(0.5, aimPower));
            shooter.setTurretPower(-aimPower);
            telemetry.addData("power", -aimPower);
            if (Math.abs(ll.pose.tx) < lastTX) {
                lastTX = ll.pose.tx;
                lastPinpoint = follower.getHeading();
                lastTurretRotation = shooter.getTTPos();
            }
        }
        else {
            telemetry.addData("last turret", lastTurretRotation);
            telemetry.addData("last pinpoint", lastPinpoint);
            shooter.setTurretPower(0);
        }

        //intake
        intake.updateSingle();
        //shooter
        shooter.shooterMachineSingle();
        
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("tt pos", shooter.getTTPos());

        telemetry.addData("tx", ll.pose.tx);
        telemetry.addData("cornerX", 39.3701*ll.pose.cornerX);
        telemetry.addData("cornerY", 39.3701*ll.pose.cornerY);

        dash.addData("shooter vel", shooter.getMotorVel());
        dash.addData("target vel", kf);
        dash.addData("flywheel rpm", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
        cameraLoopCounter += 1;
    }

    @Override
    public void stop() {
    }
}
