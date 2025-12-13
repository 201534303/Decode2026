package org.firstinspires.ftc.teamcode.pedroPathing.Old;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Auto.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous
@Disabled
public class CloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    private IntakeAuto intake;
    private DrivetrainAuto drivetrain;
    private ShooterAuto shooter;
    private ElapsedTime runtime = new ElapsedTime();

    private final Pose startPose = new Pose(125, 128, Math.toRadians(-52)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(88, 82, Math.toRadians(0));
    private final Pose ballCollect1 = new Pose(125, 88, Math.toRadians(0));
    private final Pose ballCollect2 = new Pose(125, 82, Math.toRadians(0));
    private final Pose ballCollect3 = new Pose(120, 82, Math.toRadians(0));
    //private final Pose ballCollect3 = new Pose();

    private PathChain startShoot, shootGrab1, grabShoot1, shootGrab2, grabShoot2;

    public void buildPaths(){
        startShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                //.setTimeoutConstraint(double set)//how long in ms the follower has to correct after stopping
                //.setTValueConstraint(double set)//how much of path robot must follow before complete double between 0.0 to 1.0
                //.setVelocityConstraint(double set)//velocity robot must be below in inches/second.
                //.setTranslationalConstraint(double set)//max translational error
                //.setHeadingConstraint(double set)//max heading error
                .build();
        shootGrab1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect1.getHeading())
                .build();
        grabShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose.getHeading())
                .build();
        shootGrab2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(83.000, 54.00, 0),
                                ballCollect2
                        )
                )
                //.setTimeoutConstraint(3000)
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect2.getHeading(),0.3)
                .build();

        grabShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(ballCollect2, shootPose))
                .setLinearHeadingInterpolation(ballCollect2.getHeading(), shootPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.close();
                follower.followPath(startShoot);

                if(inBetwen(getPosX(), 80, 96) && inBetwen(getPosY(), 74, 90)) {
                    telemetry.addData("IT IS: ", "BETWEEN");
                    intake.allTheWay();
                }

                //add the shoot motor current
                if(actionTimer.getElapsedTimeSeconds() > 6) {
                    intake.transferOff();
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3) {
                    intake.intakeIn();
                    follower.followPath(shootGrab1,0.4, true);
                    if(actionTimer.getElapsedTimeSeconds() > 2) {
                        intake.allTheWay();
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 2.5) {
                        intake.transferOff();
                        setPathState(2);
                    }
                }
                else{
                    intake.intakeOff();
                }
                break;
            case 2:
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabShoot1,0.8, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(shootGrab2,0.4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(grabShoot2,0.8, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.resetTimer();
    }

    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        drivetrain = new DrivetrainAuto(hardwareMap, telemetry);
    }

    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    public void start() {
        runtime.reset();
        pathTimer.resetTimer();
        setPathState(0);
    }

    public double getPosX(){ return follower.getPose().getX();}

    public boolean inBetwen(double pos, double lower, double higher){ return lower <= pos && pos <= higher; }

    public double getPosY(){ return follower.getPose().getY(); }

}


