package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class TestPaths extends Paths{
    private Follower follower;

    public Pose startPose = new Pose(125, 128, Math.toRadians(-52)); // Start Pose of our robot.
    public Pose shootPose = new Pose(88, 82, Math.toRadians(0));
    public Pose ballCollect1 = new Pose(125, 82, Math.toRadians(0));
    public Pose ballCollect2 = new Pose(125, 82, Math.toRadians(0));
}
