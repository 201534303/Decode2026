package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

abstract class Paths {
    public Follower follower;

    public static Pose makePos(double x, double y, double degrees ){//java should support default
        return new Pose(x, y, Math.toRadians(degrees));
    }
    public static Pose makePos(double x, double y){
        return new Pose(x, y, 0);
    }

    public PathChain curve(Pose pos1, Pose pos2, double x, double y) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                pos1,
                                new Pose(x,y),
                                pos2
                        )
                )
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
    }

    public double getPosX(){ return follower.getPose().getX();}

    public boolean inBetween(double pos, double lower, double higher){ return lower <= pos && pos <= higher; }

    public double getPosY(){ return follower.getPose().getY(); }

    //no way this works
    public boolean inBetween(double lowX, double hiX, double lowY, double hiY){
        //return 80 <= 87 && 96 >= 87 && 74 <= 81, 90 >= 81
        return lowX <= getPosX() && hiX >= getPosX() && lowY <= getPosY() && hiY >= getPosY() ;
    }
}
