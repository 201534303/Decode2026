package org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
public class OLDClosePaths extends Paths2 {
    public OLDClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(125, 128, -52); // Start Pose of our robot.
    public Pose shootPose = makePos(100, 86);
    public Pose ballCollect1 = makePos(133, 85);

    public Pose reset = makePos(135, 77, 0);
    public Pose resetMiddle = new Pose(116.135, 84.992);
//    public Pose reset = makePos(130, 80, 90);
//    public Pose resetMiddle = new Pose(116.135, 84.992);
    public Pose resetMiddle2 = new Pose(115, 65);
    public Pose reset2 = makePos(133, 72, -90);
    public Pose ballCollect2 = makePos(143, 60);//135, 50

    public Pose midBack = makePos(109, 56);

    public Pose shootTo2Middle = new Pose(70, 47);
    public Pose ballCollect3 = makePos(140, 38);
    public Pose ballCollect3Middle = new Pose(80, 30);//26.930
    public Pose ballCollect4 = makePos(138, 8, -90);//135, 10
    public Pose ballCollectMiddle4 = makePos(138, 30, -90);//143, 20
    public Pose shootTo5Middle = new Pose(92, 5);
    public Pose backUp = makePos(130,25,-90);
    public Pose goUp = makePos(130,5,-90);
    public Pose out = makePos(115, 90, 90);//park
    public Pose outDown = makePos(130, 100, 90);//park

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {
            this.startPose = startPose.mirror();
            this.shootPose = shootPose.mirror();
            this.ballCollect1 = ballCollect1.mirror();
            this.reset = reset.mirror();
            this.resetMiddle = resetMiddle.mirror();
            this.resetMiddle2 = resetMiddle2.mirror();
            this.reset2 = reset2.mirror();
            this.ballCollect2 = ballCollect2.mirror();
            this.midBack = midBack.mirror();
            this.shootTo2Middle = shootTo2Middle.mirror();
            this.ballCollect3 = ballCollect3.mirror();
            this.ballCollect3Middle = ballCollect3Middle.mirror();
            this.ballCollect4 = ballCollect4.mirror();
            this.ballCollectMiddle4 = ballCollectMiddle4.mirror();
            this.shootTo5Middle = shootTo5Middle.mirror();
            this.backUp = backUp.mirror();
            this.goUp = goUp.mirror();
            this.out = out.mirror();
            this.outDown = outDown.mirror();
            return true;
        }
        return false;
    }

    public PathChain shootToOut(){
        return bezierLine(shootPose, out);
    }

    public PathChain shootToOutDown(){
        return bezierLine(shootPose, outDown);
    }

    public PathChain startToShoot(){
        return bezierLine(startPose, shootPose);
    }

    public PathChain resetToShoot(){
        return bezierLine(reset, shootPose);
    }

    public PathChain threeToShoot(){
        return bezierCurve(ballCollect3,
                ballCollect3Middle,
                shootPose);
    }
    public PathChain fourToShoot(){
        return bezierLine(ballCollect4, shootPose);
    }
    public PathChain twoToShoot(){
        return bezierCurve(ballCollect2,
                midBack,
                shootPose);
    }

    public PathChain reset(){
        return follower.pathBuilder()
                .addPath(new BezierCurve(ballCollect1, resetMiddle, reset))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), reset.getHeading())
                .setTValueConstraint(.97)
                .build();
    }
    public PathChain reset2(){
        return bezierCurve(ballCollect2, resetMiddle2, reset2);
    }

    public PathChain collectToShoot(){
        final Pose ballCollect = follower.getPose();
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect, shootPose))
                .setLinearHeadingInterpolation(ballCollect.getHeading(), shootPose.getHeading())
                .setTValueConstraint(.98)
                .build();
    }

    public PathChain shootTo_(int mark){
        switch (mark){
            case 1:
                return shootTo1();
            case 2:
                return shootTo2();
            case 3:
                return shootTo3();
            case 4:
                return shootTo4();
            case 5:
                return shootTo5();
            default:
                return shootToOut();
        }
    }

    public PathChain shootTo1(){
        return bezierLine(shootPose, ballCollect1);
    }

    public PathChain shootTo2(){
        return bezierCurve(shootPose, shootTo2Middle, ballCollect2);//75.114, 52.551
    }

    public PathChain shootTo3(){
        return bezierCurve(shootPose, ballCollect3Middle, ballCollect3);
    }

    public PathChain shootTo4Mid(){
        return bezierLine(shootPose, ballCollectMiddle4);
    }

    public PathChain shootTo4(){
        return bezierLine(ballCollectMiddle4, ballCollect4);
    }

    public PathChain shootTo5(){
        return bezierCurve(shootPose, shootTo5Middle, ballCollect4);
    }

    public PathChain down(){
        return bezierLine(ballCollect4, backUp);
    }
    public PathChain up(){
        return bezierLine(backUp, goUp);
    }
}
