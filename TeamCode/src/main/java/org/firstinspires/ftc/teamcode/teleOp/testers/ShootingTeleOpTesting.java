package org.firstinspires.ftc.teamcode.teleOp.testers;

import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.OLDConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Shooter Testing", group="Iterative OpMode")
@Config
//@Disabled
public class ShootingTeleOpTesting extends OpMode {

    //choose
    private OLDChoose choose;
    private boolean shooterOff = false;

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
    private OLDChoose.Alliance currentColor = RED;
    private double x;
    private double y;
    private double heading;
    private Vector vel;

    @Override
    public void init() {
        //choose
        choose = new OLDChoose(gamepad1, telemetry);


        //localization
        follower = OLDConstants.createFollower(hardwareMap);

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, overallRuntime);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry, new Feedback(hardwareMap, new ElapsedTime(), telemetry, gamepad1, gamepad2));
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
        /*
        --------------------------GRAB COORDINATES--------------------------
         */
        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        vel = follower.getVelocity();

        double dist = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            double delX = -x;
            double delY = 144-y;
            dist = Math.hypot(delX, delY);
        }
        if(currentColor == OLDChoose.Alliance.RED){
            double delX = 144-x;
            double delY = 144-y;
            dist = Math.hypot(delX, delY);
        }


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
            if(currentColor == OLDChoose.Alliance.RED){
                currentColor = OLDChoose.Alliance.BLUE;
            }
            else{
                currentColor = OLDChoose.Alliance.RED;
            }
        }

        robot.fieldCentricDrive(currentColor, heading, vel);


        /*
        --------------------------DRIVER TWO CONTROLS--------------------------
         */

        robot.updateIntake();
        robot.updateTransfer();


        if(gamepad2.dpadUpWasPressed()){
            robot.DELETEBUTTHISISVEL += 5;
        }
        if(gamepad2.dpadDownWasPressed()){
            robot.DELETEBUTTHISISVEL -= 5;
        }

        if(gamepad2.dpadRightWasPressed()){
            robot.DELETEBUTTHISISHOOD += .05;
        }
        if(gamepad2.dpadLeftWasPressed()){
            robot.DELETEBUTTHISISHOOD -= .05;
        }

        if(gamepad2.yWasPressed()){
            robot.DELETEBUTTHISISTURRET += 0.0025;
        }
        if(gamepad2.aWasPressed()){
            robot.DELETEBUTTHISISTURRET -= 0.0025;
        }
        if(gamepad2.bWasPressed()){
            robot.DELETEBUTTHISISTURRET += 0.05;
        }
        if(gamepad2.xWasPressed()){
            robot.DELETEBUTTHISISTURRET -= 0.05;
        }

        if(gamepad2.rightBumperWasPressed()){
            shooterOff = !shooterOff;
        }



        /*
        --------------------------UPDATE--------------------------
         */
        double nowTime = overallRuntime.time(TimeUnit.SECONDS);
        double timeDif = nowTime - lastTime;
        double hertz = 1.0/timeDif;

        lastTime = nowTime;

        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("position", "(" + Math.round(x*100)/100.0 + "," + Math.round(y*100)/100.0 + ") Heading: " + Math.round(heading*100)/100.0);
        telemetry.addData("distance", Math.round(dist*100.0)/100.0);
        telemetry.addData("hertz", hertz);

        robot.updateShooterTesting(shooterOff);

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
