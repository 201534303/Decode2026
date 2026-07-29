package org.firstinspires.ftc.teamcode.teleOp.testers;

import static org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;


@TeleOp(name="turret test", group="Iterative OpMode")
@Config
public class HoodTester extends OpMode {

    //choose
    private OLDChoose choose;
    private double mul = 1.0;

    //runtime
    private ElapsedTime overallRuntime;
    private double lastTime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Feedback feedback;

    //localization
    private Follower follower;

    //robot
    private RobotActions robot;
    public boolean turretOn = true;
    private OLDChoose.Alliance currentColor = RED;
    private double x;
    private double y;
    private double heading;
    private Vector vel;
    private double counter = 0;
    private double timeSinceLastLochalazationReset = 0;
    private boolean moving;
    private boolean rotating;
    private boolean movingOrRotating;
    LimelightProcessor_v3Tele ll;
    private Telemetry dash;
    public static double kf = 0.59;
    private double timeDif = 1.0;
    private double oldHeading = 0;

    @Override
    public void init() {
        ll = new LimelightProcessor_v3Tele(hardwareMap);
        //choose
        choose = new OLDChoose(gamepad1, telemetry);


        //localization
        follower = Constants.createFollower(hardwareMap);

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, overallRuntime);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);
        feedback = new Feedback(hardwareMap, overallRuntime, telemetry, gamepad1, gamepad2);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry, feedback);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
        currentColor = RED;
        choose.allianceInit();
        currentColor = choose.getSelectedAlliance();
        telemetry.update();
    }

    @Override
    public void start() {
        overallRuntime.reset();
        if(currentColor == RED){
            follower.setPose(new Pose(115, 70, Math.PI/2));
        }
        if(currentColor == BLUE){
            follower.setPose(new Pose(29, 70, Math.PI/2));
        }
    }

    @Override
    public void loop() {
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = nowTime - lastTime;
        double hertz = 1.0/timeDif;
        lastTime = nowTime;

        ll.updateTele(follower.getPose().getHeading(), robot.turAngle, movingOrRotating);
        /*
        telemetry.addLine("------");
        telemetry.addLine("angles");
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("raw tx", ll.pose.roll);
        telemetry.addData("tx", ll.pose.tx);
        telemetry.addData("id", ll.pose.id);
        telemetry.addData("yaw", Math.toDegrees(ll.pose.yaw));
        telemetry.addLine("----------");
        telemetry.addData("distance", ll.pose.distance);
        telemetry.addData("dx",ll.pose.posX2);
        telemetry.addData("dy", ll.pose.posY2);
        telemetry.addData("rawX", ll.pose.rawX);
        telemetry.addData("rawY", ll.pose.rawY);
        telemetry.addLine("----------");
        telemetry.addData("fieldX", ll.pose.posX);
        telemetry.addData("fieldY", ll.pose.posY);
        telemetry.addLine("------");

         */
        /*
        --------------------------GRAB COORDINATES--------------------------
         */
        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        vel = follower.getVelocity();

        if (vel.getMagnitude() < 0.5) {
            moving = false;
        } else {
            moving = true;
        }
        if (Math.abs(Math.toDegrees((oldHeading - heading))/timeDif) < .06) {
            rotating = false;
        } else {
            rotating = true;
        }
        telemetry.addData("rotating", rotating);
        /*
        --------------------------DRIVER ONE CONTROLS--------------------------
         */

        //reset localization to back
        if (gamepad1.share){
            robot.setLocalizationBack();
        }

        //reset imu to 0
        if (gamepad1.options){
            robot.setIMUZero(x, y, currentColor);
        }

        //reset position to corner
        if (gamepad1.dpad_down){
            robot.setLocalizationOurSide(currentColor);
        }

        if (gamepad1.dpad_up && ll.pose.valid && !rotating && !moving) {
            if (counter > 5) {
                follower.setPose(new Pose(ll.pose.posX, ll.pose.posY, follower.getPose().getHeading()));
                feedback.notifyRelocalized();
                counter = 0;
            }
        }
        else {
            counter++;
        }

        //turn turret on/off
        if (gamepad1.y){
            turretOn = !turretOn;
        }

        //switch alliances
        if(gamepad1.xWasPressed()) {
            if(currentColor == OLDChoose.Alliance.RED){
                currentColor = OLDChoose.Alliance.BLUE;
            }
            else{
                currentColor = OLDChoose.Alliance.RED;
            }
        }

        //drive
        robot.fieldCentricDrive(currentColor, heading, vel);


        /*
        --------------------------DRIVER TWO CONTROLS--------------------------
         */

        robot.updateIntake();
        robot.updateTransfer(currentColor, vel, x, y, rotating);
        if(gamepad2.leftBumperWasPressed()){
            robot.toggleSingleDriver();
        }


        /*
        --------------------------UPDATE--------------------------
         */
        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("position", "(" + Math.round(x*100)/100.0 + "," + Math.round(y*100)/100.0 + ") Heading: " + Math.round(heading*100)/100.0);
        telemetry.addData("hertz", hertz);

        robot.updateConversion(currentColor, turretOn, x, y, heading, vel, kf, mul);

        telemetry.addData("mul", mul);

        if(gamepad2.dpadDownWasPressed()){
            mul -= .05;
        }
        if(gamepad2.dpadUpWasPressed()){
            mul += .05;
        }

        follower.update();


        feedback.updateBallState(intake.haveBall());
        telemetry.update();
        dash.update();

        movingOrRotating = moving || rotating;
        oldHeading = heading;
    }

    @Override
    public void stop() {
    }
}
