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
    public Pose startPose = makePos(80, 8); // Start Pose of our robot.
    public Pose shootPose = makePos(80, 18, 10);
    private Pose ballCollect1 = makePos(130, 45.67);
    private Pose ballCollect2 = makePos(130, 10, -90);
    public Pose ballCollect3 = makePos(125, 7);
    public Pose out3 = makePos(117, 10);
    public Pose parkPose = makePos(100, 10);

    public boolean bluePath(Choose.Alliance alliance) {
        if (alliance == Choose.Alliance.BLUE) {
            this.startPose = makePos(64, 8,180);
            this.shootPose = makePos(64, 18,170);
            this.ballCollect3 = makePos(19, 9,180);
            this.out3 = makePos(27, 10,180);
            this.parkPose = makePos(44, 10,180);
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

    public PathChain shootTo3(){
        return bezierLine(shootPose, ballCollect3);
    }

    public PathChain Out3(){
        return bezierLine(ballCollect3, out3);
    }
    public PathChain In3(){
        return bezierLine(out3, ballCollect3);
    }
}
