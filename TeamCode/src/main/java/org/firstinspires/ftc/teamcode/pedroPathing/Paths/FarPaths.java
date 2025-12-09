package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarPaths extends Paths{
    //Paths
//    public Pose start = new Pose(0, 0, Math.toRadians(0));
//    public Pose shootPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose start= new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose shootPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose ballCollect1 = new Pose(0, 0, Math.toRadians(0));
    private final Pose ballCollect2 = new Pose(0, 0, Math.toRadians(0));
    private final Pose ballCollect3 = new Pose(0, 0, Math.toRadians(0));
    //private final Pose ballCollect3 = new Pose();


//    public bluePath() {
//        //this.follower = r.f;
//
//        //if (r.a.equals(Alliance.RED)) {
//            start = start.mirror();
//    }

    public PathChain startToShoot(){
        return follower.pathBuilder()
                .addPath(new BezierLine(start, shootPose))
                .setLinearHeadingInterpolation(start.getHeading(), shootPose.getHeading())
                .build();
    }
}
