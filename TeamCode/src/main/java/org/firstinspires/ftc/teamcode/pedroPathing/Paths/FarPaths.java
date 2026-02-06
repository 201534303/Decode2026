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
    public Pose ballCollect1 = makePos(130, 35);
    public Pose ballCollect1Mid1 = new Pose(86.53307392996108, 40.95914396887157);
    public Pose ballCollect1Mid2 = new Pose(110.5603112840467, 34.48832684824902);
    public Pose shootPose = makePos(95, 12);
    public Pose ballCollect2 = makePos(133, 9);
    public Pose out = makePos(123, 9);

    public Pose park = makePos(102, 14, 0);

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

    public PathChain shootTo1(){
        return bezierCurve(shootPose,
                ballCollect1Mid1,
                ballCollect1Mid2,
                ballCollect1);
    }

    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }

    public PathChain out(){
        return bezierLine(ballCollect2, out);
    }
    public PathChain in(){
        return bezierLine(out, ballCollect2);
    }


}
