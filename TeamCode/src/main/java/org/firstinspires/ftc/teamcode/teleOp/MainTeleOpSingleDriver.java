package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;
import org.firstinspires.ftc.teamcode.JaviVision.v3.LimelightProcessor_v3;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
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

    ShooterTele shooter;
    private Telemetry dash;
    public static double power;
    public static double targetVelo = 1300;
    public double cameraLoopCounter = 0;

    //LimelightProcessor_v3 ll;

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
        //ll = new LimelightProcessor_v3(hardwareMap, odo);



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


        dt.feildCentricDrive(Math.toDegrees(follower.getPose().getHeading()));
        //dt.updateOdo();


        //intake
        intake.updateSingle();

        //shooter
        shooter.shooterMachineSingle();
        shooter.setTurretAngle(-(90-Math.toDegrees(Math.atan((144-Math.abs(follower.getPose().getY()))/follower.getPose().getX()))));

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("turr heating", (90-Math.toDegrees(Math.atan((144-Math.abs(follower.getPose().getY()))/follower.getPose().getX()))));
        telemetry.addData("raw a tan", Math.toDegrees(Math.atan((144-Math.abs(follower.getPose().getY()))/follower.getPose().getX())));

        dash.addData("shooter vel", shooter.getMotorVel());
        dash.addData("target vel", targetVelo);
        dash.addData("flywheel rpm", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
        cameraLoopCounter += 1;
    }

    @Override
    public void stop() {
    }
}
