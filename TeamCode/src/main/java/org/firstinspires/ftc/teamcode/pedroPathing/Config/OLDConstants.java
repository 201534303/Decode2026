package org.firstinspires.ftc.teamcode.pedroPathing.Config;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Compatibility shim for older Pedro Pathing autos that still import the removed
 * pedroPathing2 Config package.
 */
public final class OLDConstants {
    private OLDConstants() {}

    public static Follower createFollower(HardwareMap hardwareMap) {
        return org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
    }
}
