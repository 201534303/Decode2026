package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "ShooterTestingTwo", group = "Iterative OpMode")
@Config
public class ShooterTestingTwo extends OpMode {

    private OLDChoose choose;
    private ElapsedTime overallRuntime;
    private double lastTime;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Feedback feedback;

    private Follower follower;

    private RobotActions robot;
    public boolean turretOn = true;
    private OLDChoose.Alliance currentColor = RED;
    private double x;
    private double y;
    private double heading;
    private Vector vel;
    private double counter = 0;
    private boolean moving;
    private boolean rotating;
    private boolean movingOrRotating;
    LimelightProcessor_v3Tele ll;
    private Telemetry dash;
    public static double kf = 0.59;
    private double timeDif = 1.0;
    private double oldHeading = 0;

    @Override
    public void init() {
        ll = new LimelightProcessor_v3Tele(hardwareMap);
        choose = new OLDChoose(gamepad1, telemetry);

        follower = Constants.createFollower(hardwareMap);
        overallRuntime = new ElapsedTime();

        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, overallRuntime);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);
        feedback = new Feedback(hardwareMap, overallRuntime, telemetry, gamepad1, gamepad2);

        telemetry.addData("Status", "Initialized");

        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry, feedback);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
        currentColor = RED;
        choose.allianceInit();
        currentColor = choose.getSelectedAlliance();
        feedback.showAllianceSelection(currentColor == BLUE);
        telemetry.update();
    }

    @Override
    public void start() {
        overallRuntime.reset();
        if (PoseSaver.hasPose) {
            follower.setPose(new Pose(PoseSaver.x, PoseSaver.y, PoseSaver.heading));
            PoseSaver.clear();
        } else {
            if (currentColor == RED) {
                follower.setPose(new Pose(72, 72, 0));
            }
            if (currentColor == BLUE) {
                follower.setPose(new Pose(72, 72, Math.PI));
            }
        }
    }

    @Override
    public void loop() {
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = nowTime - lastTime;
        lastTime = nowTime;
        telemetry.addData("loop time", timeDif);

        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        vel = follower.getVelocity();

        moving = vel.getMagnitude() >= 0.5;
        rotating = Math.abs(Math.toDegrees((oldHeading - heading)) / timeDif) >= .005;
        ll.updateTele(heading, 0, movingOrRotating);

        if (gamepad1.share) {
            robot.setLocalizationBack();
        }

        if (gamepad1.options) {
            robot.setIMUZero(x, y, currentColor);
        }

        if (gamepad1.dpad_down) {
            robot.setLocalizationOurSide(currentColor);
            feedback.notifyRelocalized();
        }

        if (gamepad1.dpad_up && ll.pose.valid && !rotating && !moving) {
            if (counter > 5) {
                follower.setPose(new Pose(ll.pose.posX, ll.pose.posY, heading));
                feedback.notifyRelocalized();
                counter = 0;
            }
        } else {
            counter++;
        }

        if (gamepad1.yWasPressed()) {
            turretOn = !turretOn;
        }

        if (gamepad1.xWasPressed()) {
            if (currentColor == OLDChoose.Alliance.RED) {
                currentColor = OLDChoose.Alliance.BLUE;
            } else {
                currentColor = OLDChoose.Alliance.RED;
            }
        }

        robot.fieldCentricDrive(currentColor, heading, vel);

        robot.updateIntake();
        if (gamepad1.dpadLeftWasPressed()) {
            robot.toggleSingleDriver();
        }
        if (gamepad2.dpadDownWasPressed()) {
            robot.toggleLiftMode();
        }

        updateShooterTuningControls();

        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("position", "(" + Math.round(x * 100) / 100.0 + "," + Math.round(y * 100) / 100.0 + ") Heading: " + Math.round(heading * 10000) / 10000.0);
        telemetry.addData("dist", Math.round(100.0 * Math.hypot(144 - x, 144 - y)) / 100.0);
        robot.update(currentColor, turretOn, x, y, heading, vel, kf, false);
        telemetry.addData("shooter tune controls", "g2 y/a speed +/-5, b/x hood +/-0.05, dpad left/right flank +/-1");
        telemetry.addData(
                "shooter velocity",
                "base %.2f offset %.2f applied %.2f",
                robot.getLastBaseShooterVelocity(),
                robot.getShooterVelocityOffset(),
                robot.getLastAppliedShooterVelocity()
        );
        telemetry.addData(
                "hood",
                "base %.2f offset %.2f applied %.2f",
                robot.getLastBaseHood(),
                robot.getHoodOffset(),
                robot.getLastAppliedHood()
        );
        telemetry.addData(
                "flank position",
                "base %.2f offset %.2f applied %.2f",
                robot.getBaseFlankPosition(),
                robot.getFlankPositionOffset(),
                robot.getLastAppliedFlankPosition()
        );
        follower.update();

        feedback.updateBallState(intake.haveBall());
        telemetry.update();
        dash.update();

        movingOrRotating = moving || rotating;
        oldHeading = heading;
    }

    private void updateShooterTuningControls() {
        if (gamepad2.yWasPressed()) {
            robot.adjustShooterVelocityOffset(5.0);
        }
        if (gamepad2.aWasPressed()) {
            robot.adjustShooterVelocityOffset(-5.0);
        }
        if (gamepad2.xWasPressed()) {
            robot.adjustHoodOffset(-0.05);
        }
        if (gamepad2.bWasPressed()) {
            robot.adjustHoodOffset(0.05);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.adjustFlankPositionOffset(1.0);
        }
        if (gamepad2.dpadRightWasPressed()) {
            robot.adjustFlankPositionOffset(-1.0);
        }
    }
}
