package org.firstinspires.ftc.teamcode.auto.util;

public class PoseSaver {

    public static double x = 0.0;
    public static double y = 0.0;
    public static double heading = 0.0; // radians
    public static boolean hasPose = false;

    public static void save(double xIn, double yIn, double headingIn) {
        x = xIn;
        y = yIn;
        heading = headingIn;
        hasPose = true;
    }

    public static void clear() {
        hasPose = false;
    }
}
