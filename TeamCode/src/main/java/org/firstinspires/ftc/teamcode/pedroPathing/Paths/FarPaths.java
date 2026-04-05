package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;

public class FarPaths extends Paths {
    private static final double DETECTION_COLLECT_Y_MIN = 9.0;
    private static final double DETECTION_COLLECT_Y_MAX = 35.0;
    private static final double DETECTION_FALLBACK_Y = 12.0;

    public FarPaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(88, 8); // Start Pose of our robot
    public Pose ballCollect1 = makePos(130, 37); // 130
    public Pose ballCollect12 = makePos(133, 37);
    public Pose ballCollect1Out = makePos(120, 37);
    public Pose ballCollect1Mid = new Pose(83, 42);
    public Pose shootPose = new Pose(93, 12, 0);
    public Pose shootPose2 = new Pose(89, 17, 0);
    public Pose ballCollect2 = makePos(135, 9);
    public Pose ballCollect22 = makePos(133, 9);
    public Pose out = makePos(125, 9);
    public Pose park = makePos(105, 10, 0);
    public Pose midShoot4 = new Pose(85, 7);

    public boolean bluePath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollect12 = mirror(ballCollect12);
            ballCollect1Out = mirror(ballCollect1Out);
            ballCollect1Mid = mirror(ballCollect1Mid);
            shootPose = mirror(shootPose);
            shootPose2 = mirror(shootPose2);
            ballCollect2 = mirror(ballCollect2);
            ballCollect22 = mirror(ballCollect22);
            out = mirror(out);
            park = mirror(park);
            midShoot4 = mirror(midShoot4);
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
        return fromCurrentPose(shootPose2);
    }

    public PathChain collectToShoot2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(ballCollect22, shootPose2))
                .setLinearHeadingInterpolation(ballCollect22.getHeading(), shootPose2.getHeading())
                .build();
    }

    public PathChain to(Pose pos) {
        return follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, pos))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), pos.getHeading())
                .build();
    }

    public PathChain fromTo(Pose pos) {
        return fromCurrentPose(pos);
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
    public PathChain shootTo12() {
        return bezierCurve(follower.getPose(),
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo2() {
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootTo3() {
        return bezierLine(shootPose2, ballCollect12);
    }

    public PathChain shootTo3NotSet() {
        return fromCurrentPose(ballCollect12);
    }

    public PathChain shootTo4() { //return bezierLine(shootPose2, ballCollect2);
        return bezierCurve(shootPose2, midShoot4, ballCollect2);
    }

    public PathChain shootTo4NotSet() { //return bezierLine(shootPose2, ballCollect2);
        return bezierCurve(follower.getPose(), midShoot4, ballCollect2);
    }

    public PathChain shootToPark() {
        return bezierLine(shootPose2, park);
    }

    public PathChain outSet() {
        return bezierLine(ballCollect2, out);
    }

    public PathChain inSet() {
        return bezierLine(out, ballCollect2);
    }

    public Pose detectionCollectPose(double averageOffset, OLDChoose.Alliance alliance) {
        double collectY = clamp(shootPose2.getY() + averageOffset, DETECTION_COLLECT_Y_MIN, DETECTION_COLLECT_Y_MAX);
        Pose collectPose = new Pose(130, collectY, 0);
        if (alliance == OLDChoose.Alliance.BLUE) {
            collectPose = mirror(collectPose);
        }
        return collectPose;
    }

    public boolean shouldUseDetectionFallback(Pose collectPose) {
        return collectPose.getY() < DETECTION_FALLBACK_Y;
    }

    private double clamp(double value, double lower, double upper) {
        return Math.max(lower, Math.min(upper, value));
    }
}
