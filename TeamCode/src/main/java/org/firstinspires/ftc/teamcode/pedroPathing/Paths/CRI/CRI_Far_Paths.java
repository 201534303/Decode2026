package org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Paths;

public class CRI_Far_Paths extends Paths {
    private static final double DETECTION_COLLECT_Y_MIN = 10.0;
    private static final double DETECTION_COLLECT_Y_MAX = 35.0;
    private static final double DETECTION_COLLECT_Y_MAX_Other = 55;

    public CRI_Far_Paths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(111, 7); // Start Pose of our robot
    public Pose ballCollect1 = makePos(180, 40); // 130
    public Pose ballCollect12 = makePos(180, 40);
    public Pose ballCollect1Mid = new Pose(131, 37);
    public Pose shootPose = new Pose(120, 17, 0); // 93, 12, 0
    public Pose ballCollect2 = makePos(180, 8);
    public Pose ballCollect2Middle = makePos(130.99027237354085, 3.0492866407263293);

    public Pose spike3 = makePos(180, 55);
    public Pose spike3mid = makePos(122.17736705577174, 62.41958495460444);
    private Pose spike3mid2 = makePos(140.09727626459144, 52.250972762645915);

    public Pose park = makePos(130, 17, 0);

    public Pose park2 = makePos(130, 34, 0);
    public Pose middleShoot = makePos(123, 34);

    public boolean bluePath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollect12 = mirror(ballCollect12);
            ballCollect1Mid = mirror(ballCollect1Mid);
            shootPose = mirror(shootPose);
            ballCollect2 = mirror(makePos(180, 7));
            ballCollect2Middle = mirror(ballCollect2Middle);
            spike3mid = mirror(spike3mid);
            spike3mid2 = mirror(spike3mid2);
            spike3 = mirror(spike3);
            park = mirror(park);
            park2 = mirror(park2);
            middleShoot = mirror(middleShoot);
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

    public PathChain collectToShootNotSet() {
        return fromCurrentPose(shootPose);
    }

    public PathChain collectToShootNotSetMid() {
        return fromCurrentPose(middleShoot);
    }

    public PathChain to(Pose pos) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pos))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pos.getHeading())
                .build();
    }

    public PathChain toOther(Pose pos) {
        return follower.pathBuilder()
                .addPath(new BezierLine(middleShoot, pos))
                .setLinearHeadingInterpolation(middleShoot.getHeading(), pos.getHeading())
                .build();
    }

    public PathChain to(Pose pos1, Pose pos2) {
        return follower.pathBuilder()
                .addPath(new BezierLine(pos1, pos2))
                .setLinearHeadingInterpolation(pos1.getHeading(), pos2.getHeading())
                .build();
    }

    public PathChain shootTo1() {
        return bezierCurve(startPose,
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootToSpike3() {
        return bezierCurve(shootPose,
                spike3mid,
                spike3mid2,
                spike3);
    }

    public PathChain shootTo2() {
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootTo3() {
        return bezierLine(shootPose, ballCollect12);
    }

    public PathChain shootTo3Other() {
        return bezierLine(middleShoot, spike3);
    }

    public PathChain shootTo4Other() {
        return bezierLine(middleShoot, ballCollect2);
    }

    public PathChain shootTo4OtherOther() {
        return bezierCurve(middleShoot, ballCollect2Middle, ballCollect2);
    }

    public PathChain shootTo4() {
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootToPark() {
        return bezierLine(shootPose, park);
    }

    public PathChain shootToPark2() {
        return bezierLine(middleShoot, park2);
    }

    public Pose detectionCollectPose(double averageOffset, OLDChoose.Alliance alliance) {
        double collectY = clamp(shootPose.getY() + averageOffset, DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX);
        Pose collectPose = new Pose(185, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public Pose detectionCollectPoseOther(double averageOffset, OLDChoose.Alliance alliance) {
        double collectY = clamp(shootPose.getY() + averageOffset, DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX_Other);
        Pose collectPose = new Pose(185, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public boolean shouldUseDetectionFallback(Pose collectPose) {
        return DETECTION_COLLECT_Y_MIN >= collectPose.getY();
    }

    public boolean shouldUseDetectionFallbackOther(Pose collectPose) {
        return DETECTION_COLLECT_Y_MIN >= collectPose.getY();
    }

    private double clamp(double value, double lower, double upper) {
        return Math.max(lower, Math.min(upper, value));
    }

    public Pose detectionCollectPoseNotSet(double averageOffset, OLDChoose.Alliance alliance) {
        Pose currPose = follower.getPose();

        double collectY = clamp(currPose.getY() + (averageOffset * 2), DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX);
        Pose collectPose = new Pose(176, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public Pose detectionCollectPoseNotSetOther(double averageOffset, OLDChoose.Alliance alliance) {
        Pose currPose = follower.getPose();

        double collectY = clamp(currPose.getY() + (averageOffset * 2), DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX_Other);
        Pose collectPose = new Pose(176, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }
}
