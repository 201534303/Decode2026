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

    public final Pose startPose = makePos(125, 128, -52); // Start Pose of our robot.
    private final Pose shootPose = makePos(100, 82);
    public final Pose ballCollect1 = makePos(128, 82);
    public final Pose reset = makePos(133, 70, 90);
    public final Pose ballCollect2 = makePos(134, 50);
    public final Pose ballCollect3 = makePos(134, 28);
    public final Pose ballCollect4 = makePos(135, 16, -90);//135, 10
    public final Pose ballCollectMiddle4 = makePos(130, 30, -90);//135, 10
    public final Pose backUp = makePos(130,25,-90);
    public final Pose goUp = makePos(130,5,-90);
    public final Pose out = makePos(115, 70);

    public PathChain shootToOut(){
        return bezierLine(shootPose, out);
    }

    public PathChain startToShoot(){
        return bezierLine(startPose, shootPose);
    }

    public PathChain reset(){
        return bezierCurve(ballCollect1, new Pose(116.135, 74.992), reset);
    }

    public PathChain collectToShoot(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain shootTo_(int mark){
        switch (mark){
            case 1:
                return shootTo1();
            case 2:
                return shootTo2();
            case 3:
                return shootTo3();
            case 4:
                return shootTo4();
            case 5:
                return shootTo5();
            default:
                return shootToOut();
        }
    }

    public PathChain shootTo1(){
        return bezierLine(shootPose, ballCollect1);
    }

    public PathChain shootTo2(){
        return bezierCurve(shootPose, new Pose(75.114, 52.551), ballCollect2);
    }

    public PathChain shootTo3(){
        return bezierCurve(shootPose, new Pose(65.730, 26.930), ballCollect3);
    }

    public PathChain shootTo4Mid(){
        return bezierLine(shootPose, ballCollectMiddle4);
    }

    public PathChain shootTo4(){
        return bezierLine(ballCollectMiddle4, ballCollect4);
    }

    public PathChain shootTo5(){
        return bezierCurve(shootPose, new Pose(92, 5), ballCollect4);
    }

    public PathChain down(){
        return bezierLine(ballCollect4, backUp);
    }
    public PathChain up(){
        return bezierLine(backUp, goUp);
    }
}
