/*package org.firstinspires.ftc.teamcode.JaviVision;

import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
@Autonomous
public class Limelight_Test {
    private Limelight3A l;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;
    public Limelight_Test(HardwareMap hardwareMap) {
        l = hardwareMap.get(Limelight3A.class, "limelight");
        l.pipelineSwitch(6);
        l.start();
    }

    public void start() {
        l.start();
    }

    public void stop() {
        l.stop();
    }

    public void pause() {
        l.pause();
    }

    public double distanceFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 8; // right/left from tag
            double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8; // forward/back from tag

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }

        return 0;
    }

    public double distanceFromBlue() {
        return distanceFromTag(20);
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }

    public double angleFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null)
            return target.getTargetXDegrees();

        return 0;
    }

    public double angleFromBlue() {
        return angleFromTag(20);
    }

    public double angleFromRed() {
        return angleFromTag(24);
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.setPollRateHz(20);
        l.start();
    }
}

 */