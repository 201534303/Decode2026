package org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
public class OLDClosePaths extends Paths2 {
    public OLDClosePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(125, 128, -52); // Start Pose of our robot.
    public Pose shootPose = makePos(100, 82);
    public Pose ballCollect1 = makePos(135, 85);
    public Pose reset = makePos(133, 82, 90);
    public Pose resetMiddle = new Pose(116.135, 84.992);
    public Pose resetMiddle2 = new Pose(115, 65);
    public Pose reset2 = makePos(133, 72, -90);
    public Pose ballCollect2 = makePos(150, 60);//135, 50
    public Pose shootTo2Middle = new Pose(70, 47);
    public Pose ballCollect3 = makePos(145, 38);
    public Pose ballCollect3Middle = new Pose(60, 26.930);
    public Pose ballCollect4 = makePos(140, 8, -90);//135, 10
    public Pose ballCollectMiddle4 = makePos(130, 30, -90);//143, 20
    public Pose shootTo5Middle = new Pose(92, 5);
    public Pose backUp = makePos(130,25,-90);
    public Pose goUp = makePos(130,5,-90);
    public Pose out = makePos(115, 70, 90);//park
    public Pose outDown = makePos(130, 100, 90);//park

    public boolean bluePath(OLDChoose.Alliance alliance) {
        if (alliance == OLDChoose.Alliance.BLUE) {
            this.startPose = makePos(19, 128, 232);
            this.shootPose = makePos(44, 82,180);
            this.ballCollect1 = makePos(14, 82,180);
            this.reset = makePos(11, 72, 90);
            this.resetMiddle = new Pose(27.865, 70,180);
            this.resetMiddle2 = resetMiddle2.mirror();
            this.reset2 = reset2.mirror();
            this.ballCollect2 = makePos(2, 50,180);
            this.shootTo2Middle = new Pose(83, 47,180);
            this.ballCollect3 = makePos(2, 28,180);
            this.ballCollect3Middle = new Pose(78.27, 26.93,180);
            this.ballCollect4 = ballCollect4.mirror();  //makePos(14, 8, 270);//135, 10
            this.ballCollectMiddle4 = ballCollectMiddle4.mirror();//makePos(10, 20, 270);//130, 30
            this.shootTo5Middle = new Pose(52, 5,180);
            this.backUp = makePos(14,25,270);
            this.goUp = makePos(14,5,270);
            this.out = makePos(29, 70,90); // park
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

    public PathChain reset(){
        return bezierCurve(ballCollect1, resetMiddle, reset);
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
