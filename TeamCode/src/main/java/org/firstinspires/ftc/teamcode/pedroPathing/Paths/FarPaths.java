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
    private Pose shootPose = makePos(80, 18, 5);
    private Pose ballCollect1 = makePos(130, 45.67);
    private Pose ballCollect2 = makePos(130, 10, -90);
    private Pose ballCollect3 = makePos(125, 7);
    private Pose out3 = makePos(117, 10);
    private Pose parkPose = makePos(100, 10);

    public void bluePath(Choose.Alliance alliance) {
        if (alliance == Choose.Alliance.BLUE) {
            startPose = startPose.mirror();
            shootPose = shootPose.mirror();
            ballCollect1 = ballCollect1.mirror();
            ballCollect2 = ballCollect2.mirror();
            ballCollect3 = ballCollect3.mirror();
            out3 = out3.mirror();
            parkPose = parkPose.mirror();
        }
    }

    public  PathChain startToShoot(){
        return bezierLine(startPose, shootPose);
    }
    public PathChain shootToOut(){
        return bezierLine(shootPose, parkPose);
    }

    public PathChain shootTo1(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(82.366, 44.366),
                                new Pose(99.922, 46.607),
                                ballCollect1
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

    public PathChain shootTo2(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(115.518, 46.506),
                                new Pose(125.506, 40.342),
                                ballCollect2
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
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
