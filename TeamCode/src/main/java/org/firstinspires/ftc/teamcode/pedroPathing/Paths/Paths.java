package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

abstract class Paths {
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
                .setLinearHeadingInterpolation(pos1.getHeading(), pos3.getHeading())
                .build();
    }

    public PathChain bezierLine(Pose pos1, Pose pos2){
        return follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
    }

    public double getPosX(){ return follower.getPose().getX();}
    public double getPosY(){ return follower.getPose().getY(); }

    public boolean inBetween(double pos, double lower, double higher){ return lower <= pos && pos <= higher; }
    public boolean inBetween(double lowX, double hiX, double lowY, double hiY){
        //return 80 <= 87 && 96 >= 87 && 74 <= 81, 90 >= 81
        return lowX <= getPosX() && hiX >= getPosX() && lowY <= getPosY() && hiY >= getPosY() ;
    }
}
