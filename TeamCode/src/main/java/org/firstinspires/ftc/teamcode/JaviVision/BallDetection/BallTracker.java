package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

import java.util.ArrayList;
import java.util.List;

public class BallTracker {

    private final List<TrackedBall> balls = new ArrayList<>();
    private int nextId = 1;

    // ===== TUNE THESE =====
    private static final double MAX_MATCH_DIST_SQ = 500 * 500;
    private static final int    MAX_FRAMES_LOST   = 1000;

    /**
     * Call every frame with raw detections [[x, y], [x, y], ...] and
     * the time in milliseconds since the last call.
     */
    public void update(List<double[]> detections, double timeDifIn) {
        double timeDif = timeDifIn / 1000.0;

        List<TrackedBall> aliveBalls = new ArrayList<>();
        for (TrackedBall b : balls) {
            if (b.alive || b.framesLost < 20) aliveBalls.add(b);
        }


        boolean[] ballMatched = new boolean[aliveBalls.size()];
        boolean[] detMatched  = new boolean[detections.size()];

        // Build pairs using predicted positions
        List<double[]> pairs = new ArrayList<>();
        for (int i = 0; i < detections.size(); i++) {
            for (int j = 0; j < aliveBalls.size(); j++) {
                TrackedBall ball = aliveBalls.get(j);
                double predictedX = ball.x + ball.vx * timeDif;
                double predictedY = ball.y + ball.vy * timeDif;
                double dx = detections.get(i)[0] - predictedX;
                double dy = detections.get(i)[1] - predictedY;
                pairs.add(new double[]{i, j, dx * dx + dy * dy});
            }
        }

        pairs.sort((a, b) -> Double.compare(a[2], b[2]));

        for (double[] pair : pairs) {
            if (pair[2] > MAX_MATCH_DIST_SQ) break;

            int detIdx  = (int) pair[0];
            int ballIdx = (int) pair[1];
            if (detMatched[detIdx] || ballMatched[ballIdx]) continue;

            TrackedBall ball = aliveBalls.get(ballIdx);
            double newX = detections.get(detIdx)[0];
            double newY = detections.get(detIdx)[1];

           /* if (ball.kalmanX == null) ball.kalmanX = new KalmanFilter(newX, 0.1, 0.3);
            double smoothedX = ball.kalmanX.update(newX);
            ball.x = smoothedX;*/

            if (newX != ball.x || newY != ball.y) {
                // Fresh position from Limelight — compute velocity over the full stale window
                ball.timeSinceLastFreshMs += timeDifIn;
                double freshTimeDif = ball.timeSinceLastFreshMs / 1000.0;
                ball.vx = (newX - ball.lastFreshX) / freshTimeDif;
                ball.vy = (newY - ball.lastFreshY) / freshTimeDif;
                android.util.Log.d("TRACKER", "VELOCITY ball=" + ball.id
                        + " vx=" + ball.vx + " vy=" + ball.vy
                        + " dt=" + freshTimeDif);
                ball.lastFreshX = newX;
                ball.lastFreshY = newY;
                ball.timeSinceLastFreshMs = 0;
            } else {
                // Same value as last frame — Limelight hasn't updated yet, just accumulate time
                ball.timeSinceLastFreshMs += timeDifIn;
            }

            android.util.Log.d("TRACKER", "MATCH det=" + detIdx + " ball=" + ball.id
                    + " newX=" + newX + " oldX=" + ball.x);

            ball.x = newX;
            ball.y = newY;
            ball.alive = true;
            ball.framesLost = 0;

            detMatched[detIdx]   = true;
            ballMatched[ballIdx] = true;
        }

        // Unmatched alive balls → coast forward
        for (int j = 0; j < aliveBalls.size(); j++) {
            if (!ballMatched[j]) {
                TrackedBall ball = aliveBalls.get(j);
                ball.x += ball.vx * timeDif;
                ball.y += ball.vy * timeDif;
                ball.alive = false;
                ball.framesLost++;
            }
        }

        // Unmatched detections → new ball
        for (int i = 0; i < detections.size(); i++) {
            if (!detMatched[i]) {
                android.util.Log.d("TRACKER", "NEW BALL id=" + nextId + " x=" + detections.get(i)[0]);
                TrackedBall b = new TrackedBall(nextId++, detections.get(i)[0], detections.get(i)[1]);
                b.lastFreshX = b.x;
                b.lastFreshY = b.y;
                b.timeSinceLastFreshMs = 0;
                balls.add(b);
            }
        }

        balls.removeIf(b -> !b.alive && b.framesLost > MAX_FRAMES_LOST);
    }

    public List<TrackedBall> getAllBalls() { return balls; }

    public List<TrackedBall> getAliveBalls() {
        List<TrackedBall> alive = new ArrayList<>();
        for (TrackedBall b : balls) { if (b.alive) alive.add(b); }
        return alive;
    }
}