package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class ClosePaths extends Paths{
    public ClosePaths(Follower follower) {
        this.follower = follower;
    }
    //Paths
    //private final Pose startPose = makePos(125, 128, -52);// Start Pose of our robot.
    //private final Pose shootPose = makePos(88, 82);

    public final Pose startPose = new Pose(125, 128, Math.toRadians(-52)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(88, 82, Math.toRadians(0));
    private final Pose ballCollect1 = makePos(125, 88);
    private final Pose ballCollect2 = makePos(125, 82);
    private final Pose ballCollect3 = makePos(120, 82);

    public PathChain startToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain shootTo1(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect1.getHeading())
                .build();
    }

    public PathChain _1ToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain collectToShoot(){
        //test this out
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .build();
    }
    public PathChain shootToCollect(Pose ballCollect){ //test this out
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect.getHeading())
                .build();
    }

}
