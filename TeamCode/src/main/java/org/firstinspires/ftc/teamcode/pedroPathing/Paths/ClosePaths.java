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

    public Pose startPose = makePos(126, 120, 35); // Start Pose of our robot.
    public Pose shootPose0 = makePos(85, 80, 35);
    public Pose shootPose = makePos(88, 77);
    public Pose shootPose2 = makePos(85, 90);
    public Pose ballCollect1 = makePos(130, 55, 0);
    public Pose ballCollectMid1 = new Pose(90, 50);
    public Pose ballCollectMid2 = new Pose(126.74319066147861, 63.48054474708172);
    public Pose selfee = makePos(130, 60, 35);
    public Pose selfeeWiggle = makePos(127, 57, 35);
    public Pose selfeeMid = new Pose(92.9805447470817, 47.437743190661486);
    public Pose ballCollect2 = makePos(125, 80);
    public Pose park = makePos(110, 80, 90);

    public Pose reset = makePos(120, 72, 90);
    public Pose resetMiddle = new Pose(116.135, 74.992);

    public PathChain reset(){
        return bezierCurve(ballCollect1, resetMiddle, reset);
    }

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {
            startPose = startPose.mirror(); // Start Pose of our robot.
            shootPose0 = shootPose0.mirror();
            shootPose = shootPose.mirror();
            shootPose2 = shootPose2.mirror();
            ballCollect1 = ballCollect1.mirror();
            ballCollectMid1 = ballCollectMid1.mirror();
            ballCollectMid2 = ballCollectMid2.mirror();
            selfee = selfee.mirror();
            selfeeWiggle = selfeeWiggle.mirror();
            selfeeMid = selfeeMid.mirror();
            ballCollect2 = ballCollect2.mirror();
            park = park.mirror();
            reset = reset.mirror();
            resetMiddle = resetMiddle.mirror();

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
    public PathChain selfeeToShoot(){
        return bezierCurve(selfee,
                ballCollectMid2,
                ballCollectMid1,
                shootPose);
    }

    public PathChain fistToShoot(){
        return bezierLine(startPose, shootPose0);
    }
    public PathChain shootTo1(){
        //return bezierLine(shootPose0, ballCollect1);
        return bezierCurve(shootPose0,
                ballCollectMid1,
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

    public PathChain _2ToShoot(){
        return bezierLine(ballCollect2, shootPose);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }
}
