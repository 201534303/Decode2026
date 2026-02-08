package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;

public class FarPaths extends Paths{
    public FarPaths(Follower follower){
        this.follower = follower;
    }

    public Pose startPose = makePos(87, 8); // Start Pose of our robot
    public Pose ballCollect1 = makePos(130, 37);
    public Pose ballCollect1Out = makePos(120, 37);
    public Pose ballCollect1Mid = new Pose(83, 42);
    public Pose shootPose = makePos(93, 12);
    public Pose shootPose2 = makePos(90, 17);
    public Pose ballCollect2 = makePos(130, 9);
    public Pose out = makePos(120, 9);

    public Pose park = makePos(98, 25, 90);

    private Pose outGet;
    private Pose ballCollectVision;
    private Pose ballCollectOut;
    OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;

    public boolean bluePath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.BLUE) {
            this.alliance = getAlliance;
            startPose = startPose.mirror();
            ballCollect1 = ballCollect1.mirror();
            ballCollect1Out = ballCollect1Out.mirror();
            ballCollect1Mid = ballCollect1Mid.mirror();
            shootPose = shootPose.mirror();
            shootPose2 = shootPose2.mirror();
            ballCollect2 = ballCollect2.mirror();
            out = out.mirror();
            park = park.mirror();
            return true;
        }
        return false;
    }

    public PathChain collectToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain collectToShoot3(){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose2))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose2.getHeading())
                .build();
    }

    public PathChain collectToShoot2(){
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect2, shootPose2))
                .setLinearHeadingInterpolation(ballCollect2.getHeading(), shootPose2.getHeading())
                .build();
    }

    public PathChain to(Pose pos){
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, pos))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), pos.getHeading())
                .build();
    }

    public PathChain set(double x, double y){
        final Pose currPose = follower.getPose();
        final Pose goTo = new Pose(x, y);

        return follower.pathBuilder()
                .addPath(new BezierLine(currPose, goTo))
                .setLinearHeadingInterpolation(currPose.getHeading(), currPose.getHeading())
                .build();
    }

    public PathChain shootTo1(){
        return bezierCurve(startPose,
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo12(){
        return bezierCurve(follower.getPose(),
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo2(){ return bezierLine(shootPose, ballCollect2); }
    public PathChain shootTo3(){ return bezierLine(shootPose2, ballCollect1); }
    public PathChain shootToPark(){ return bezierLine(shootPose2, park); }

    public PathChain outSet(){ return bezierLine(ballCollect2, out); }

    public PathChain outFrom(Pose pos1, Pose pos2){
        return bezierLine(pos1, pos2);
    }
    public PathChain inFrom(Pose pos1, Pose pos2){
        return bezierLine(pos1, pos2);
    }


    public PathChain inSet(){ return bezierLine(out, ballCollect2); }

    public PathChain outSet2(){ return bezierLine(ballCollect1, ballCollect1Out); }

    public PathChain inSet2(){ return bezierLine(ballCollect1Out, ballCollect1); }
}
