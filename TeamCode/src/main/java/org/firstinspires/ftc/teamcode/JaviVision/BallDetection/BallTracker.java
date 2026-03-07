package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import java.util.ArrayList;
import java.util.List;

public class BallTracker {

    private final List<TrackedBall> balls = new ArrayList<>();
    private int nextId = 1;

    // ===== TUNE THESE =====
    private static final double MAX_MATCH_DIST_SQ = 10 * 10; // max distance (squared) to match a detection to a ball
    private static final int    MAX_FRAMES_LOST   = 10;        // frames before a lost ball is removed entirely

    /**
     * Call every frame with raw detections [[x, y], [x, y], ...] and
     * the time in seconds since the last call.
     */
    public void update(List<double[]> detections, double timeDif) {

        // Only match against alive balls
        List<TrackedBall> aliveBalls = new ArrayList<>();
        for (TrackedBall b : balls) {
            if (b.alive) aliveBalls.add(b);
        }

        boolean[] ballMatched = new boolean[aliveBalls.size()];
        boolean[] detMatched  = new boolean[detections.size()];

        // Build all (detIdx, ballIdx, distSq) triples using predicted positions
        List<double[]> pairs = new ArrayList<>();
        for (int i = 0; i < detections.size(); i++) {
            for (int j = 0; j < aliveBalls.size(); j++) {
                TrackedBall ball = aliveBalls.get(j);

                // Predict where this ball should be now based on velocity
                double predictedX = ball.x + ball.vx * timeDif;
                double predictedY = ball.y + ball.vy * timeDif;

                double dx = detections.get(i)[0] - predictedX;
                double dy = detections.get(i)[1] - predictedY;
                double distSq = dx * dx + dy * dy;

                pairs.add(new double[]{i, j, distSq});
            }
        }

        // Sort by distance ascending
        pairs.sort((a, b) -> Double.compare(a[2], b[2]));

        // Greedy 1-to-1 assignment (closest pairs first)
        for (double[] pair : pairs) {
            if (pair[2] > MAX_MATCH_DIST_SQ) break; // sorted, so everything after is too far

            int detIdx  = (int) pair[0];
            int ballIdx = (int) pair[1];
            if (detMatched[detIdx] || ballMatched[ballIdx]) continue;

            TrackedBall ball = aliveBalls.get(ballIdx);
            double newX = detections.get(detIdx)[0];
            double newY = detections.get(detIdx)[1];

            // Update velocity (units per second) before updating position
            ball.vx = (newX - ball.x) / timeDif;
            ball.vy = (newY - ball.y) / timeDif;

            ball.x = newX;
            ball.y = newY;
            ball.alive = true;
            ball.framesLost = 0;

            detMatched[detIdx]   = true;
            ballMatched[ballIdx] = true;
        }

        // Unmatched alive balls → mark lost, coast forward using velocity
        for (int j = 0; j < aliveBalls.size(); j++) {
            if (!ballMatched[j]) {
                TrackedBall ball = aliveBalls.get(j);
                ball.x += ball.vx * timeDif; // extrapolate position while lost
                ball.y += ball.vy * timeDif;
                ball.alive = false;
                ball.framesLost++;
            }
        }

        // Unmatched detections → brand new ball
        for (int i = 0; i < detections.size(); i++) {
            if (!detMatched[i]) {
                balls.add(new TrackedBall(nextId++, detections.get(i)[0], detections.get(i)[1]));
            }
        }

        // Remove balls that have been lost too long
        balls.removeIf(b -> !b.alive && b.framesLost > MAX_FRAMES_LOST);
    }

    /** All balls currently being tracked (alive and recently lost) */
    public List<TrackedBall> getAllBalls() {
        return balls;
    }

    /** Only balls visible in the most recent frame */
    public List<TrackedBall> getAliveBalls() {
        List<TrackedBall> alive = new ArrayList<>();
        for (TrackedBall b : balls) {
            if (b.alive) alive.add(b);
        }
        return alive;
    }
}