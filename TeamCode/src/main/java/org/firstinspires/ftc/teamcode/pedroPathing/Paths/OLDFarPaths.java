package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class OLDFarPaths extends Paths{
    public OLDFarPaths(Follower follower) {
        this.follower = follower;
    }
    public Pose startPose = makePos(80, 8); // Start Pose of our robot.
    public Pose shootPose = makePos(80, 18, 10);
    public Pose ballCollect = makePos(125, 8);
    public Pose out = makePos(117, 10);

    public Pose outSide = makePos(110, 20);
    public Pose ballCollectSide = makePos(125, 20);

    public Pose outSideInSetup = makePos(80, 10);
    public Pose outSideIN = makePos(110, 8);
    public Pose ballCollectSideIN = makePos(125, 8);
    public Pose parkPose = makePos(75, 40, 90);

    public boolean bluePath(Choose.Alliance alliance) {
        if (alliance == Choose.Alliance.BLUE) {
            this.startPose = makePos(64, 8, 180);
            this.shootPose = makePos(64, 18, 170);
            this.ballCollect = makePos(17.5, 9, 180);
            this.out = makePos(27, 10, 180);
            this.outSide = makePos(34, 20, 180);
            this.ballCollectSide = makePos(19, 20, 180);
            this.outSideInSetup = makePos(64, 10, 180);
            this.outSideIN = makePos(34, 8, 180);
            this.ballCollectSideIN = makePos(17.5, 8, 180);
            this.parkPose = parkPose.mirror();

            return true;
        }
        return false;
    }

    public  PathChain startToShoot(){
        return bezierLine(startPose, shootPose);
    }
    public PathChain shootToOut(){
        return bezierLine(shootPose, parkPose);
    }

    public PathChain collectToShoot(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain shootTo(){
        return bezierLine(shootPose, ballCollect);
    }
    public PathChain shootToIn(){
        return bezierLine(shootPose, ballCollectSideIN);
    }

    public PathChain Out(){
        return bezierLine(ballCollect, out);
    }
    public PathChain In(){
        return bezierLine(out, ballCollect);
    }

    public PathChain shootToSide(){
        return bezierLine(shootPose, ballCollectSide);
    }
    public PathChain OutSide(){
        return bezierLine(ballCollect, outSide);
    }
    public PathChain InSide(){
        return bezierLine(out, ballCollectSide);
    }

    public PathChain outSideSetup() {
        return bezierLine(shootPose, outSideInSetup);
    }
    public PathChain OutSideIn(){
        return bezierLine(ballCollect, outSideIN);
    }
    public PathChain InSideIn(){
        return bezierLine(out, ballCollectSideIN);
    }
    public PathChain InSideInOther(){
        return bezierLine(outSideInSetup, ballCollectSideIN);
    }
}
