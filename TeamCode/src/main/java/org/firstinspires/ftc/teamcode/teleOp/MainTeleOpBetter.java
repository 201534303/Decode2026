package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.v3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Two Driver Tele", group="Iterative OpMode")
@Config
public class MainTeleOpBetter extends OpMode {

    //choose
    private Choose choose;

    //runtime
    private ElapsedTime overallRuntime;
    private double lastTime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private double turretAngle = 0.48;

    //localization
    private Follower follower;

    //robot
    private RobotActions robot;
    public boolean turretOn = true;
    private Choose.Alliance currentColor = RED;
    private double x;
    private double y;
    private double heading;
    private Vector vel;
    private double counter = 0;
    private boolean moving;
    private boolean rotating;
    private boolean movingOrRotating;
    private double storedTurAngle = 0;
    LimelightProcessor_v3Tele ll;

    @Override
    public void init() {
        ll = new LimelightProcessor_v3Tele(hardwareMap);
        //choose
        choose = new Choose(gamepad1, telemetry);


        //localization
        follower = Constants.createFollower(hardwareMap);

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry);
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
        ll.updateTele(follower.getPose().getHeading(), robot.turAngle, movingOrRotating);
        if (10 < getRuntime() && getRuntime() <= 11) {
            counter++;
        }
        else {
            telemetry.addData("num runs in a second", counter);
        }
        telemetry.addData("new turret", robot.turAngle);
        telemetry.addData("old turret", storedTurAngle);
        telemetry.addData("rotating", rotating);
        telemetry.addData("movingOrRotating", movingOrRotating);
        telemetry.addLine("------");
        telemetry.addLine("ANGLES");
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("theta (no alpha)", ll.pose.x);
        telemetry.addData("tx", Math.toDegrees(ll.pose.tx));
        telemetry.addData("tx (no alpha", ll.pose.y);
        telemetry.addLine("----------");
        telemetry.addData("distance", ll.pose.distance);
        telemetry.addData("distance (no alpha)", ll.pose.z);
        telemetry.addData("rawX", ll.pose.rawX);
        telemetry.addData("rawY", ll.pose.rawY);
        telemetry.addLine("----------");
        telemetry.addData("fieldX", ll.pose.posX);
        telemetry.addData("fieldY", ll.pose.posY);
        telemetry.addData("positionX", follower.getPose().getX());
        telemetry.addData("positionY", follower.getPose().getY());
        telemetry.addLine("------");
        /*
        --------------------------GRAB COORDINATES--------------------------
         */
        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        vel = follower.getVelocity();

        /*
        --------------------------DRIVER ONE CONTROLS--------------------------
         */

        //reset localization to back
        if (gamepad1.share){
            robot.setLocalizationBack();
        }

        //reset imu to 0
        if (gamepad1.options){
            follower.setPose(new Pose(ll.pose.posX, ll.pose.posY, follower.getPose().getHeading()));
            //robot.setIMUZero(currentColor, x, y);
        }

        //rezero position
        if (gamepad1.dpad_down){
            robot.setLocalizationOurSide(currentColor);
        }


        if (gamepad1.y){
            if (turretOn){
                turretOn = false;
            } else {
                turretOn = true;
            }
        }

        if(gamepad1.xWasPressed()) {
            if(currentColor == Choose.Alliance.RED){
                currentColor = Choose.Alliance.BLUE;
            }
            else{
                currentColor = Choose.Alliance.RED;
            }
        }


        robot.fieldCentricDrive(currentColor, heading);


        /*
        --------------------------DRIVER TWO CONTROLS--------------------------
         */

        robot.updateIntake();
        robot.updateTransfer(currentColor, vel, x, y);

        /*
        --------------------------UPDATE--------------------------
         */
        double nowTime = overallRuntime.time(TimeUnit.SECONDS);
        double timeDif = nowTime - lastTime;
        double hertz = 1.0/timeDif;
        lastTime = nowTime;
        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("position", "(" + Math.round(x*100)/100.0 + "," + Math.round(y*100)/100.0 + ") Heading: " + Math.round(heading*100)/100.0);
        telemetry.addData("hertz", hertz);
        robot.update(currentColor, turretOn, x, y, heading, vel);
        follower.update();
        telemetry.update();
        if (follower.getVelocity().getMagnitude() < 0.05) {
            moving = false;
        } else {
            rotating = true;
        }
        final double margin = Math.toRadians(2);
        if (storedTurAngle - margin < robot.turAngle && storedTurAngle + margin > robot.turAngle) {
            rotating = false;
        } else {
            rotating = true;
        }
        storedTurAngle = robot.turAngle;
        movingOrRotating = moving || rotating;
    }

    @Override
    public void stop() {
    }
}
