package org.firstinspires.ftc.teamcode.pedroPathing.OldAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;

@Autonomous
@Disabled
public class TestAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(84, 2, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose setPickPose = new Pose(100, 82, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickupPose = new Pose(127, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose shootPose = new Pose(80, 102, Math.toRadians(47)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose(59, 43.2, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    //private Path scorePreload;
    private PathChain startSet, setPick, pickShoot, shootEnd;

    public void buildPaths(){
        startSet = follower.pathBuilder()
                .addPath(new BezierLine(startPose, setPickPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), setPickPose.getHeading())
                .build();
        setPick = follower.pathBuilder()
                .addPath(new BezierLine(setPickPose, pickupPose))
                .setLinearHeadingInterpolation(setPickPose.getHeading(), pickupPose.getHeading())
                .build();
        pickShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();
        shootEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(startSet);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(setPick,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(pickShoot,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(shootEnd,true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


        @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
