package org.firstinspires.ftc.teamcode.pedroPathing2.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing2.Paths.OLD.OLDChoose;

public class ClosePaths extends Paths{
    public ClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(126, 120, 35); // Start Pose of our robot.
    public Pose shootPose0 = makePos(85, 80, 35);
    public Pose shootPose = makePos(88, 85);
    public Pose shootPose2 = makePos(90, 110);
    public Pose ballCollect1 = makePos(128, 60, 0);
    public Pose ballCollectMid1 = new Pose(90, 55);
    public Pose ballCollectMid2 = new Pose(126.74319066147861, 63.48054474708172);
    public Pose selfee = makePos(132, 60, 35);//132, 60, 35
    public Pose selfee2 = makePos(132, 60, 35);
    public Pose selfeeWiggle = makePos(127, 57, 35);
    public Pose selfeeMid = new Pose(80, 47);
    public Pose ballCollect2 = makePos(126, 90);
    public Pose park = makePos(90, 110);

    public Pose reset = makePos(120, 72, 90);
    public Pose resetMiddle = new Pose(116.135, 74.992);
    public Pose ballCollect3 = makePos(130, 40, 0);
    public Pose ballCollect3Mid = new Pose(70, 27);


    public PathChain reset(){
        return bezierCurve(ballCollect1, resetMiddle, reset);
    }

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose); // Start Pose of our robot.
            shootPose0 = mirror(shootPose0);
            shootPose = mirror(shootPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollectMid1 = mirror(ballCollectMid1);
            ballCollectMid2 = mirror(ballCollectMid2);
            selfee = new Pose(10, 61, Math.toRadians(145));
            selfee2 = new Pose(10, 61, Math.toRadians(145));
            selfeeWiggle = mirror(selfeeWiggle);
            selfeeMid = mirror(selfeeMid);
            ballCollect2 = new Pose(18, 90, Math.toRadians(180));
            park = mirror(park);
            ballCollect3 = mirror(ballCollect3);
            ballCollect3Mid = mirror(ballCollect3Mid);
            reset = mirror(reset);
            resetMiddle = mirror(resetMiddle);

            return true;
        }
        return false;
    }

    public PathChain collectToShoot(){
        return line(follower.getPose(), shootPose, .98);
    }

    public PathChain toStart(){
        return line(follower.getPose(), startPose, .98);
    }

    public PathChain ballCollect1ToShoot(){
        return bezierCurve(ballCollect1,
                ballCollectMid2,
                ballCollectMid1,
                shootPose);
    }
    public PathChain selfeeToShoot(){
        return bezierCurve(selfee,
                ballCollectMid2,
                ballCollectMid1,
                shootPose);
    }

    public PathChain firstToShoot(){
        return bezierLine(startPose, shootPose0);
    }
    public PathChain shootTo1(){
        //return bezierLine(shootPose0, ballCollect1);
        return bezierCurve(shootPose0,
                ballCollectMid1,
                ballCollect1);
    }

    public PathChain shootTo3(){
        return bezierCurve(shootPose,
                ballCollect3Mid,
                ballCollect3);
    }

    public PathChain shootToSelfee(){
        return bezierCurve(shootPose,
                selfeeMid,
                selfee);
    }

    public PathChain shootToSelfee2(){
        return bezierCurve(shootPose,
                selfeeMid,
                selfee2);
    }
    public PathChain selfeeWiggle1(){
        return bezierLine(selfee, selfeeWiggle);
    }
    public PathChain selfeeWiggle2(){
        return bezierLine(selfeeWiggle, selfee);
    }
    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain _2ToShoot(){
        return bezierLine(ballCollect2, shootPose);
    }

    public PathChain _2ToShoot2(){
        return bezierLine(ballCollect2, shootPose2);
    }

    public PathChain _3ToShoot(){
        return bezierLine(ballCollect3, shootPose2);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }

    public PathChain _ToPark(){
        return fromCurrentPose(park);
    }

}
