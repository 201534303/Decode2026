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

    // spike mark ball collection
    public Pose ballCollect1 = makePos(127, 35);
    public Pose ballCollect1Mid = new Pose(83, 39);

    //    public Pose ballCollect1Mid1 = new Pose(86.5, 41);
//    public Pose ballCollect1Mid2 = new Pose(110.5, 34.5);
    public Pose shootPose = makePos(95, 10);
    public Pose shootPose2 = makePos(90, 17);
    public Pose ballCollect2 = makePos(130, 9);
    public Pose out = makePos(123, 9);

    public Pose park = makePos(102, 14, 0);

    private Pose inOut1;
    private Pose inOut2;

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {

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

    public PathChain collectToShoot2(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose2))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose2.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain to(double x, double y){
        final Pose currPose = follower.getPose();
        double newX = 130;//currPose.getX() + x;
        double newY = currPose.getY() - y;

        if (x == 0 && y == 0){
            newX = 87;
            newY = 35;
        }

        if (newY < 9){ newY = 9; }
        if (newX > 130){ newX = 130; }

        final Pose ballCollect = new Pose(newX, newY);
        return follower.pathBuilder()
                .addPath(new BezierLine(currPose, ballCollect))
                .setLinearHeadingInterpolation(currPose.getHeading(), currPose.getHeading())
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
        return bezierCurve(shootPose,
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo12(){
        return bezierCurve(follower.getPose(),
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo2(){ return bezierLine(shootPose, ballCollect2); }
    public PathChain shootToPark(){ return bezierLine(shootPose, park); }

    public PathChain outSet(){ return bezierLine(ballCollect2, out); }

    public PathChain inSet(){ return bezierLine(out, ballCollect2); }

    public PathChain out(){
        inOut1 = follower.getPose();
        double newX = inOut1.getX() - 10;
        double newY = inOut1.getY();
        inOut2 = new Pose(newX, newY);

        return follower.pathBuilder()
                .addPath(new BezierLine(inOut1, inOut2))
                .setLinearHeadingInterpolation(0, 0)
                .build();
    }

    public PathChain in(){
        inOut1 = new Pose(inOut1.getX(), inOut1.getY());
        return bezierLine(inOut2, inOut1);
    }




}
