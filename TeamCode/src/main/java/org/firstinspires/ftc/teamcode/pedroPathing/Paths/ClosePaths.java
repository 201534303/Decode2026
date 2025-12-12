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
    //Paths
    //private final Pose startPose = makePos(125, 128, -52);// Start Pose of our robot.
    //private final Pose shootPose = makePos(88, 82);

    public final Pose startPose = new Pose(125, 128, Math.toRadians(-52)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(88, 82, Math.toRadians(0));
    public final Pose ballCollect1 = makePos(130, 82);
    public final Pose reset = new Pose(135, 75, Math.toRadians(90));
    public final Pose ballCollect2 = makePos(135, 55);
    public final Pose ballCollect3 = makePos(135, 30);
    public final Pose out = makePos(88, 60);

    public PathChain shootToOut(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, out))
                .setLinearHeadingInterpolation(shootPose.getHeading(), out.getHeading())
                .build();
    }

    public PathChain startToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain reset(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ballCollect1,
                                new Pose(116.135, 74.992),
                                reset
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
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
                                    shootPose,
                                    new Pose(76.114, 52.551),
                                    ballCollect2
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
    }

    public PathChain shootTo3(){
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(60.730, 26.930),
                                ballCollect3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain _1ToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain collectToShoot(){
        //test this out
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .build();
    }
    public PathChain shootToCollect(Pose ballCollect){ //test this out
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect.getHeading())
                .build();
    }

}
