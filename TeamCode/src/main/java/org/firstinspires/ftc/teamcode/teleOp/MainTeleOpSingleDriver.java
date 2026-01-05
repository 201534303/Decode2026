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
import org.firstinspires.ftc.teamcode.JaviVision.v4.LimelightProcessor_v4Tele;
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

    public final Pose homing = new Pose(72, 3, -Math.toRadians(90));
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
        double heading = Math.toDegrees(follower.getPose().getHeading());
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();
        double turretAngle = (Math.toDegrees(Math.atan((144 - posX) / (144 - (Math.abs(posY)))))) - (heading + 90);

        if (gamepad1.share){
            follower.setPose(homing);
        }
        reset = new Pose (posX, posY, 0);
        if (gamepad1.options){
            follower.setPose(reset);
        }
        follower.update();

        shooter.setTurretAngle(turretAngle);
        dt.feildCentricDrive(heading);

        //ll.updateTele(follower.getPose().getHeading(), shooter.getTurrentAngle());
        //ll.getRobotPose();
        telemetry.addData("distance", ll.pose.distance * 39.3701);
        telemetry.addData("posX", ll.pose.posX * 39.3701);
        telemetry.addData("posY", ll.pose.posY * 39.3701);
        telemetry.addData("tan value", Math.toDegrees(Math.atan(ll.pose.posY/ll.pose.posX)));
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("stored_shooter",Math.toDegrees(shooter.getTurrentAngle()));
        telemetry.addData("rawPosX", ll.pose.rawX * 39.3701);
        telemetry.addData("rawPosY", ll.pose.rawY * 39.3701);
        //dt.updateOdo();
        /*
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

                //telemetry.addLine("//////");
                //telemetry.addData("angle", angle);
                //telemetry.addData("current turret", Math.toDegrees(shooter.getTTPos()));
                //telemetry.addData("last turret", Math.toDegrees(lastTurretRotation));
                //telemetry.addData("current heading", Math.toDegrees(Math.abs(follower.getPose().getHeading())));
                //telemetry.addData("last heading", Math.toDegrees(Math.abs(lastPinpoint)));
                //telemetry.addLine("////");

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
        //shooter.setTurretPower(0);

        */




        //intake
        intake.updateSingle();
        //shooter
        shooter.shooterMachineSingle(Math.abs(posY));

        telemetry.addData("turret target heading", turretAngle);
        
        telemetry.addData("x", posX);
        telemetry.addData("y", posY);
        telemetry.addData("heading", heading);
        telemetry.addData("heading delta", -(heading+90));


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
