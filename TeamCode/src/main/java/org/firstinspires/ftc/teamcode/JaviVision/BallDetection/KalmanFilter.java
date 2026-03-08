package org.firstinspires.ftc.teamcode.JaviVision.BallDetection;

public class KalmanFilter {
    private double x;   // estimated state
    private double p;   // estimated covariance

    // ===== TUNE THESE =====
    private double Q;  // process noise — higher = follows measurement more
    private double R;  // sensor noise — higher = smoother but more lag

    public KalmanFilter(double initialX,double Qin, double Rin) {
        x = initialX;
        p = 1;
        Q = Qin;
        R = Rin;
    }

    public double update(double z) {
        // Predict
        p = p + Q;

        // Update
        double K = p / (p + R);
        x = x + K * (z - x);
        p = (1 - K) * p;

        return x;
    }

    public double getEstimate() { return x; }
}