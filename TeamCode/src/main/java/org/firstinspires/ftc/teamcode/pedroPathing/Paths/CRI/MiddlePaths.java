package org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Paths;

public class MiddlePaths extends Paths {
    private static final double DETECTION_COLLECT_Y_MIN = 50;
    private static final double DETECTION_COLLECT_Y_MAX = 81;

    public MiddlePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(77, 168, 216); // Start Pose of our robot
    public Pose ballCollect1 = makePos(4.5, 59, 180); // 130
    public Pose pathToCollect1 = makePos(55, 59, 180);
    public Pose pathToCollect1_1 = makePos(71.58333333333334, 98.6721789883268, 180);
    public Pose pathToCollect1_2 = makePos(60, 57.03267012798878, 180);
    public Pose toNotHitGate = makePos(70, 150, 260);
    public Pose detectionCollectLow = makePos(3, 81, 180);
    public Pose detectionCollectHigh = makePos(3, 50, 180);

    public Pose shootPose = new Pose(82, 59, 180); // 93, 12, 0

    public Pose park = makePos(64, 59, 180);

    public boolean redPath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.RED) {
            startPose = mirror(startPose);
            ballCollect1 = mirror(ballCollect1);
            pathToCollect1 = mirror(pathToCollect1);
            pathToCollect1_1 = mirror(pathToCollect1_1);
            pathToCollect1_2 = mirror(pathToCollect1_2);
            toNotHitGate = mirror(toNotHitGate);
            detectionCollectLow = mirror(detectionCollectLow);
            detectionCollectHigh = mirror(detectionCollectHigh);
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

    public PathChain notHit(){
        return bezierLine(startPose, toNotHitGate);
    }
    public PathChain firstTo1(){
        return bezierCurve(
                toNotHitGate,
                pathToCollect1_1,
                pathToCollect1_2,
                pathToCollect1
        );
    }
    public PathChain collect1(){
        return bezierLine(pathToCollect1, ballCollect1);
    }

    public PathChain toHighDetect(){
        return bezierLine(shootPose, detectionCollectHigh);
    }

    public PathChain toLowDetect(){
        return bezierLine(shootPose, detectionCollectLow);
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
        Pose collectPose = new Pose(180, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public Pose detectionCollectPoseNotSet(double averageOffset, OLDChoose.Alliance alliance) {
        Pose currPose = follower.getPose();

        double collectY = clamp(currPose.getY() + (averageOffset * 2), DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX);
        Pose collectPose = new Pose(180, collectY, 0);
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
