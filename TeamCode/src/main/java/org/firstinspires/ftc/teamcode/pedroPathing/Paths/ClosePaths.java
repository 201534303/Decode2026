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
    //public final Pose startPose = new Pose(126, 118, Math.toRadians(-52)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(100, 82, Math.toRadians(0));
    public final Pose ballCollect1 = makePos(128, 82);
    public final Pose reset = new Pose(130, 75, Math.toRadians(90));
    public final Pose ballCollect2 = makePos(134, 50);
    public final Pose ballCollect3 = makePos(134, 28);
    public final Pose ballCollect4 = makePos(135, 11, -70);
    public final Pose ballCollect5 = makePos(135, 10, -90);

    public final Pose out = makePos(115, 70, 0);

    public PathChain shootToOut(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, out))
                .setLinearHeadingInterpolation(shootPose.getHeading(), out.getHeading())
                //.setTValueConstraint(.50)
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
                                    new Pose(75.114, 52.551),
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
                                new Pose(65.730, 26.930),
                                ballCollect3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public PathChain shootTo4(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect4.getHeading())
                .build();
    }

    public PathChain shootTo5(){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect5.getHeading())
                .build();
    }

    public PathChain shootTo(Pose ballCollect){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain collectToShoot(){
        //test this out
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

//    public PathChain collectToShoot(){
//        final Pose ballCollect = follower.getPose();
//        return follower
//                .pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                ballCollect,
//                                new Pose(70, ballCollect.getY()+20),
//                                shootPose
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//    }
    public PathChain shootToCollect(Pose ballCollect){ //test this out
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballCollect))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballCollect.getHeading())
                .build();
    }

}
