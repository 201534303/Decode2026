package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

public class TrackedBall {

    public final int id;
    public double x, y;
    public double vx, vy;    // units per second
    public boolean alive;
    public int framesLost;

    public TrackedBall(int id, double x, double y) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.vx = 0;
        this.vy = 0;
        this.alive = true;
        this.framesLost = 0;
    }
}