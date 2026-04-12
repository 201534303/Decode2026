package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing2.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.pedroPathing2.Paths.OLD.OLDChoose.Alliance.RED;

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
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing2.Config.OLDConstants;
import org.firstinspires.ftc.teamcode.pedroPathing2.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Two Driver Tele", group="Iterative OpMode")
@Config
public class MainTeleOpBetter extends OpMode {

    //choose
    private OLDChoose choose;

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

//89.7,5.6
    @Override
    public void init() {
        ll = new LimelightProcessor_v3Tele(hardwareMap);
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
        feedback.showAllianceSelection(currentColor == BLUE);
        telemetry.update();
    }

    @Override

    public void start() {
        overallRuntime.reset();
        if (PoseSaver.hasPose) {
            follower.setPose(new Pose(PoseSaver.x, PoseSaver.y, PoseSaver.heading));
            PoseSaver.clear(); // optional, prevents stale reuse
        } else {
            if(currentColor == RED){
                follower.setPose(new Pose(72, 72, 0));
            }
            if(currentColor == BLUE){
                follower.setPose(new Pose(72, 72, Math.PI));
            }
        }
    }

    @Override
    public void loop() {
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = nowTime - lastTime;
        lastTime = nowTime;
        telemetry.addData("loop time", timeDif);

        /*telemetry.addLine("------");
        telemetry.addLine("angles");
        telemetry.addData("theta", Math.toDegrees(ll.pose.theta));
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Cam Heading", ll.pose.heading);
        telemetry.addData("tx", ll.pose.tx);
        telemetry.addData("id", ll.pose.id);
        telemetry.addData("yaw", ll.pose.yaw);
        telemetry.addData("Median yaw", ll.pose.median_yaw);
        telemetry.addData("size", ll.pose.roll);
        telemetry.addLine("----------");
        telemetry.addData("distance", ll.pose.distance);
        telemetry.addData("rawX ", ll.pose.rawX);
        telemetry.addData("rawY", ll.pose.rawY);
        telemetry.addData("posX", ll.pose.posX);
        telemetry.addData("posY", ll.pose.posY);
        telemetry.addLine("----------");
        telemetry.addData("dx", ll.pose.dx);
        telemetry.addData("dy", ll.pose.dy);
        //telemetry.addLine("------");
         */


        /*
        --------------------------GRAB COORDINATES--------------------------
         */
        //shooter.flywheelSpinDynamicLoop();

        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        //shooter.flywheelSpinDynamicLoop();

        vel = follower.getVelocity();

        if (vel.getMagnitude() < 0.5) {
            moving = false;
        } else {
            moving = true;
        }
        if (Math.abs(Math.toDegrees((oldHeading - heading))/timeDif) < .005) {
            rotating = false;
        } else {
            rotating = true;
        }
        ll.updateTele(heading, 0, movingOrRotating);
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
            feedback.notifyRelocalized();
        }

        if (gamepad1.dpad_up && ll.pose.valid && !rotating && !moving) {
            if (counter > 5) {
                follower.setPose(new Pose(ll.pose.posX, ll.pose.posY, heading));
                feedback.notifyRelocalized();
                counter = 0;
            }
        }
        else {
            counter++;
        }

        //turn turret on/off
        if (gamepad1.yWasPressed()){
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
        if(gamepad1.dpadLeftWasPressed()){
            robot.toggleSingleDriver();
        }
        if(gamepad2.dpadDownWasPressed()){
            robot.toggleLiftMode();
        }

        /*
        --------------------------UPDATE--------------------------
         */
        //double currentVel = shooter.getMotorVel();
        //double targetVel = shooter.getTargetVelocity();
        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("position", "(" + Math.round(x*100)/100.0 + "," + Math.round(y*100)/100.0 + ") Heading: " + Math.round(heading*10000)/10000.0);
        telemetry.addData("dist", Math.round(100.0*Math.hypot(144-x, 144-y))/100.0);
        //dash.addData("current wheel speed", currentVel);
       // dash.addData("target wheel speed", targetVel);
       // dash.addData("shooter deviation", currentVel-targetVel);
        robot.update(currentColor, turretOn, x, y, heading, vel, kf);
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
