package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarPaths extends Paths{
    public FarPaths(Follower follower){
        this.follower = follower;
    }

    public Pose startPose = makePos(87, 8); // Start Pose of our robot
    public Pose ballCollect1 = makePos(134, 35);
    public Pose ballCollect1Mid1 = new Pose(86.53307392996108, 40.95914396887157);
    public Pose ballCollect1Mid2 = new Pose(110.5603112840467, 34.48832684824902);
    public Pose shootPose = makePos(87, 8);
    public Pose ballCollect2 = makePos(133, 9);
    public Pose park = makePos(87, 30, 90);

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

    public PathChain shootTo1(){
        return bezierCurve(shootPose,
                ballCollect1Mid1,
                ballCollect1Mid2,
                ballCollect1);
    }

    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }
}
