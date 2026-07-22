package org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Paths;

public class MiddlePaths extends Paths {
    private static final double DETECTION_COLLECT_Y_MIN = 62;
    private static final double DETECTION_COLLECT_Y_MAX = 78;

    public MiddlePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(126, 181, 270); // Start Pose of our robot
    public Pose ballCollect1 = makePos(186.5, 58, 0); // 130
    public Pose ballCollect2 = makePos(180, 81, 0); // 130
    public Pose ballCollect1_2 = makePos(126.55285343709471, 32.89591439688718);
    public Pose ballCollect1_3 = makePos(162.30966277561612, 65.5129701686122);
    public Pose shootPose = new Pose(120, 58, 0); // 93, 12, 0
    public Pose shootFirstPose = new Pose(128, 131, 0); // 93, 12, 0

    public Pose park = makePos(130, 58, 0);

    public boolean bluePath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollect2 = mirror(ballCollect2);
            ballCollect1_2 = mirror(ballCollect1_2);
            ballCollect1_3 = mirror(ballCollect1_3);
            shootFirstPose = mirror(shootFirstPose);
            shootPose = mirror(shootPose);
            park = mirror(park);
            return true;
        }
        return false;
    }

    public PathChain collectToShoot() {
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect1, shootPose))
                .setLinearHeadingInterpolation(ballCollect1.getHeading(), shootPose.getHeading())
                .build();
    }

    public PathChain startToShoot(){
        return bezierLine(startPose, shootFirstPose);
    }

    public PathChain shootTo1(){
        return bezierLine(shootPose, ballCollect1);
    }
    public PathChain shootTo2(){
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootToPark() {
        return bezierLine(shootPose, park);
    }

    public PathChain to(Pose pos) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pos.getHeading())
                .build();
    }

    public PathChain to(Pose pos1, Pose pos2) {
        return follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
    }

    public Pose detectionCollectPose(double averageOffset, OLDChoose.Alliance alliance) {
        double collectY = clamp(shootPose.getY() + averageOffset, DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX);
        Pose collectPose = new Pose(185, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public boolean shouldUseDetectionFallback(Pose collectPose) {
        return DETECTION_COLLECT_Y_MIN >= collectPose.getY();
    }

    private double clamp(double value, double lower, double upper) {
        return Math.max(lower, Math.min(upper, value));
    }

    public PathChain collectToShootNotSet() {
        return fromCurrentPose(shootPose);
    }
}
