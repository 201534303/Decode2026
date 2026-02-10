package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;

public class ClosePaths extends Paths{
    public ClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(118, 132, -54); // Start Pose of our robot.
    public Pose shootPose = makePos(90, 95);
    public Pose ballCollect1 = makePos(130, 63);
    public Pose ballCollectMid1 = new Pose(100, 55);
    public Pose ballCollectMid2 = new Pose(118, 64);
    public Pose selfee = makePos(130, 68, 25);
    public Pose selfeeWiggle = makePos(127, 68, 25);
    public Pose selfeeMid = new Pose(92.9805447470817, 47.437743190661486);
    public Pose ballCollect2 = makePos(125, 90);
    public Pose park = makePos(110, 80, 90);

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {

            return true;
        }
        return false;
    }

    public PathChain collectToShoot(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain toStart(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, startPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), startPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain ballCollect1ToShoot(){
        return bezierCurve(ballCollect1,
                ballCollectMid2,
                ballCollectMid1,
                shootPose);
    }

    public PathChain fistToShoot(){
        return bezierLine(startPose, shootPose);
    }
    public PathChain shootTo1(){
        return bezierCurve(shootPose,
                ballCollectMid1,
                ballCollectMid2,
                ballCollect1);
    }

    public PathChain shootToSelfee(){
        return bezierCurve(shootPose,
                selfeeMid,
                selfee);
    }
    public PathChain selfeeWiggle1(){
        return bezierLine(selfee, selfeeWiggle);
    }
    public PathChain selfeeWiggle2(){
        return bezierLine(selfeeWiggle, selfee);
    }
    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }
}
