package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;

public class ClosePaths extends Paths{
    public ClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = new Pose(121, 113, 0.6987); // 126, 120
    public Pose shootPose0 = makePos(86, 73, 35);
    public Pose shootPose = makePos(86, 75); // 83, 78
    public Pose shootPose2 = makePos(85, 103);
    public Pose ballCollect1 = makePos(131.17, 54.87, 0);
    public Pose ballCollectMid1 = new Pose(85, 53); // 55
    public Pose ballCollectMid12 = new Pose(94.72697795071335, 56.30350194552529); // 55
    public Pose ballCollectMid2 = new Pose(121.74319066147861, 56.48054474708172);
    public Pose selfee = new Pose(130, 53, Math.toRadians(30));//130, 54, Math.toRadians(30)
    public Pose selfeeMid = new Pose(90, 48);// 90, 55
    public Pose ballCollect2 = new Pose(123.56, 80.42, 0);
    public Pose park = makePos(85, 103);
    public Pose ballCollect3 = makePos(130, 33, 0);
    public Pose ballCollect3Mid1 = new Pose(87.20687418936447, 19.265888456549924);
    public Pose ballCollect3Mid2 = new Pose(122.20103761348896, 37.611867704280144);



    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose); // Start Pose of our robot.
            shootPose0 = mirror(shootPose0);
            shootPose = mirror(shootPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollectMid1 = mirror(ballCollectMid1);
            ballCollectMid2 = mirror(ballCollectMid2);
            selfee = new Pose(10, 61, Math.toRadians(145));
            selfeeMid = mirror(selfeeMid);
            ballCollect2 = new Pose(18, 90, Math.toRadians(180));
            park = mirror(park);
            ballCollect3 = mirror(ballCollect3);
            ballCollect3Mid1 = mirror(ballCollect3Mid1);
            ballCollect3Mid2 = mirror(ballCollect3Mid2);
            shootPose2 = mirror(shootPose2);

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
                ballCollectMid12,
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
                ballCollect3Mid1,
                ballCollect3Mid2,
                ballCollect3);
    }

    public PathChain shootToSelfee(){
        return bezierCurve(shootPose,
                selfeeMid,
                selfee);
    }

    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain _2ToShoot(){
        return bezierLine(ballCollect2, shootPose);
    }

    public PathChain _2ToShoot2(){ return bezierLine(ballCollect2, shootPose2); }
    public PathChain selfeeToShoot2(){
        return bezierLine(selfee, shootPose2);
    }

    public PathChain _3ToShoot(){
        return bezierLine(ballCollect3, shootPose);
    }
    public PathChain shootToPark(){
        return bezierLine(shootPose, park);
    }

    public PathChain _ToPark(){
        return fromCurrentPose(park);
    }

}
