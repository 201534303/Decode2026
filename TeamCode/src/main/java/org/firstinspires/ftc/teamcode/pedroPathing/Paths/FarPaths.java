package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarPaths extends Paths{
    public FarPaths(Follower follower) {
        this.follower = follower;
    }

    //Paths
//    public Pose start = new Pose(0, 0, Math.toRadians(0));
//    public Pose shootPose = new Pose(0, 0, Math.toRadians(0));

    public final Pose startPose = makePos(80, 8); // Start Pose of our robot.
    private final Pose shootPose = makePos(90, 20);
    private final Pose ballCollect1 = new Pose(130, 8, Math.toRadians(20));
    private final Pose ballCollect2 = makePos(130, 40);
    private final Pose ballCollect3 = makePos(130, 58);
    private final Pose outPose = makePos(80, 35);

//    public bluePath() {
//        //this.follower = r.f;
//
//        //if (r.a.equals(Alliance.RED)) {
//            start = start.mirror();
//    }

    public PathChain shootToOut(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, outPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), outPose.getHeading())
                .build();
    }

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

    public PathChain shootTo2(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(70, 40.5),
                                ballCollect2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();
    }

    public PathChain shootTo3(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(70, 70),
                                ballCollect3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain collectToShoot(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }
}
