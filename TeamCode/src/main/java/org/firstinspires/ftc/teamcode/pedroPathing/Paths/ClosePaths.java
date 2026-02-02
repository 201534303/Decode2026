package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class ClosePaths extends Paths{
    public ClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(118, 132, -54); // Start Pose of our robot.
    public Pose shootPose = makePos(95, 85);
    public Pose ballCollect1 = makePos(136, 58);
    public Pose ballCollectMid1 = new Pose(80.74513618677042, 50.84046692607004);
    public Pose ballCollectMid2 = new Pose(118.54280155642023, 56.61867704280156);
    public Pose selfee = makePos(130, 58, 30);
    public Pose selfeeMid = new Pose(92.9805447470817, 47.437743190661486);
    public Pose ballCollect2 = makePos(127, 85);
    public Pose park = makePos(120, 72, 90);

    public boolean bluePath(Choose.Alliance alliance) {
        if (alliance == Choose.Alliance.BLUE) {

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

    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }
}
