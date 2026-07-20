package org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Paths;

public class MiddlePaths extends Paths {
    private static final double DETECTION_COLLECT_Y_MIN = 15.0;
    private static final double DETECTION_COLLECT_Y_MAX = 35.0;

    public MiddlePaths(Follower follower) {
        this.follower = follower;
    }

    public Pose startPose = makePos(111, 7); // Start Pose of our robot
    public Pose ballCollect1 = makePos(183, 40); // 130
    public Pose ballCollect12 = makePos(183, 40);
    public Pose ballCollect1Out = makePos(173, 40);
    public Pose ballCollect1Mid = new Pose(131, 37);
    public Pose shootPose = new Pose(120, 17, 0); // 93, 12, 0
    public Pose ballCollect2 = makePos(185, 7);
    public Pose out = makePos(175, 7);

    public Pose park = makePos(132, 9, 0);
    private Pose outForDeteciton = new Pose();

    public boolean bluePath(OLDChoose.Alliance getAlliance) {
        if (getAlliance == OLDChoose.Alliance.BLUE) {
            startPose = mirror(startPose);
            ballCollect1 = mirror(ballCollect1);
            ballCollect12 = mirror(ballCollect12);
            ballCollect1Out = mirror(ballCollect1Out);
            ballCollect1Mid = mirror(ballCollect1Mid);
            shootPose = mirror(shootPose);
            ballCollect2 = mirror(ballCollect2);
            out = mirror(out);
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

    public PathChain collectToShootNotSet() {
        return fromCurrentPose(shootPose);
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

    public PathChain shootTo1() {
        return bezierCurve(startPose,
                ballCollect1Mid,
                ballCollect1);
    }

    public PathChain shootTo2() {
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootTo3() {
        return bezierLine(shootPose, ballCollect12);
    }

    public PathChain shootTo4() {
        return bezierLine(shootPose, ballCollect2);
    }

    public PathChain shootToPark() {
        return bezierLine(shootPose, park);
    }

    public PathChain outSet() {
        return bezierLine(ballCollect2, out);
    }

    public PathChain outNotSet(Pose ballCollect, OLDChoose.Alliance alliance) {
        if(alliance == OLDChoose.Alliance.BLUE) {
            outForDeteciton = new Pose(ballCollect.getX() + 10, ballCollect.getY(), ballCollect.getHeading());
        } else {
            outForDeteciton = new Pose(ballCollect.getX() - 10, ballCollect.getY(), ballCollect.getHeading());
        }
        return bezierLine(ballCollect, outForDeteciton);
    }

    public PathChain inNotSet(Pose ballCollect) {
        return bezierLine(outForDeteciton, ballCollect);
    }

    public PathChain inSet() {
        return bezierLine(out, ballCollect2);
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
}
