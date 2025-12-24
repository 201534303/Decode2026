package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarPaths extends Paths{
    public FarPaths(Follower follower) {
        this.follower = follower;
    }
    public final Pose startPose = makePos(80, 8); // Start Pose of our robot.
    private final Pose shootPose = makePos(90, 20);
    private final Pose ballCollect1 = makePos(130, 8, 20);
    private final Pose ballCollect2 = makePos(130, 40);
    private final Pose ballCollect3 = makePos(130, 58);
    private final Pose outPose = makePos(80, 35);

//    public bluePath() {
//        this.follower = r.f;
//
//        if (r.a.equals(Choose.Alliance.RED)) {
//            start = start.mirror();
//        }
//    }

    public PathChain shootToOut(){
        return bezierLine(shootPose, outPose);
    }

    public PathChain startToShoot(){
        return bezierLine(startPose, shootPose);
    }

    public PathChain shootTo1(){
        return bezierLine(shootPose, ballCollect1);
    }

    public PathChain shootTo2(){
        return bezierCurve(startPose, new Pose(70, 40.5), ballCollect2);
    }

    public PathChain shootTo3(){
        return bezierCurve(shootPose, new Pose(70, 70), ballCollect3);
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
