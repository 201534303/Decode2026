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
    private boolean moving = false;
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
        ll.updateTele();
        ll.getRobotPose(follower.getPose().getHeading(), robot.angle, moving);
        telemetry.addLine("------");
        telemetry.addData("distance", ll.pose.distance * 39.3701);
        telemetry.addData("rawX", ll.pose.rawX * 39.3701);
        telemetry.addData("rawY", ll.pose.rawY * 39.3701);
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("stored_shooter", robot.angle);
        telemetry.addData("tx", Math.toDegrees(ll.pose.tx));
        telemetry.addData("heading", ll.pose.heading);
        telemetry.addData("'true heading'", follower.getPose().getHeading());
        telemetry.addData("x", ll.pose.x * 39.3701);
        telemetry.addData("y", ll.pose.y * 39.3701);
        telemetry.addData("z", ll.pose.z * 39.3701);
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
            robot.setIMUZero(currentColor, x, y);
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

        if(gamepad1.xWasPressed()){
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
    }

    @Override
    public void stop() {
    }
}
