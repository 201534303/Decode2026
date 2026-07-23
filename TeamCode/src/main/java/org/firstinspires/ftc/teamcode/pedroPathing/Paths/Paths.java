package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class Paths {
    public Follower follower;

    public static Pose makePos(double x, double y, double degrees){
        return new Pose(x, y, Math.toRadians(degrees));
    }
    public static Pose makePos(double x, double y){
        return new Pose(x, y, 0);
    }

    public PathChain bezierCurve(Pose pos1, Pose pos2, Pose pos3) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(pos1, pos2, pos3))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pos1.getHeading(), pos3.getHeading())
                .build();
    }

    public PathChain bezierCurveTan(Pose pos1, Pose pos2, Pose pos3) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(pos1, pos2, pos3))
                //.setTangentHeadingInterpolation()
                .setTangentHeadingInterpolation()
                .build();
    }

    public PathChain bezierCurveTan(Pose pos1, Pose pos2, Pose pos3, Pose pos4) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(pos1, pos2, pos3, pos4))
                .setTangentHeadingInterpolation()
                .build();
    }

//    public PathChain bezierCurve(Pose pos1, Pose pos2, Pose pos3) {
//        return follower.pathBuilder()
//                .addPath(new BezierCurve(pos1, pos2, pos3))
//                .setLinearHeadingInterpolation(pos1.getHeading(), pos3.getHeading())
//                .build();
//    }
    public PathChain bezierCurve(Pose pos1, Pose pos2, Pose pos3, Pose pos4) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(pos1, pos2, pos3, pos4))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos4.getHeading())
                .build();
    }

    public PathChain bezierLine(Pose pos1, Pose pos2){
        return follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
    }

    public PathChain bezierLineTan(Pose pos1, Pose pos2){
        return follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setTangentHeadingInterpolation()
                .build();
    }

    protected PathChain fromCurrentPose(Pose pos1) {
        return bezierLine(follower.getPose(), pos1);
    }

    protected PathChain line(Pose start, Pose end, double tValueConstraint) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .setTValueConstraint(tValueConstraint)
                .build();
    }

    protected Pose mirror(Pose pose) {
        return pose.mirror();
    }

    public double getPosX(){ return follower.getPose().getX();}
    public double getPosY(){ return follower.getPose().getY(); }
}
