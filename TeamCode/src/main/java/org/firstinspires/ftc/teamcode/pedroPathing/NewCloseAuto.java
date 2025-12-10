package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class NewCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    private ClosePaths paths;

    //robot stuff
    private IntakeAuto intake;
    private DrivetrainAuto drivetrain;
    private ShooterAuto shooter;
    private ElapsedTime runtime = new ElapsedTime();
    private int counter;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.close();
                follower.followPath(paths.startToShoot());

                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && paths.inBetween(80, 96, 74, 90)) {
                    telemetry.addData("IT IS: ", "BETWEEN");
                    intake.allTheWay();
                    setPathState(2);
                }

                break;

//            case 2:
//                //shooter check
//                if (paths.inBetween(80, 96, 74, 90)) {
//                    telemetry.addData("IT IS: ", "BETWEEN");
//                    intake.allTheWay();
//                }
//
//                break;

//                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 3) {
//                    intake.intakeIn();
//                    follower.followPath(paths.shootTo1());
//
//                    if(actionTimer.getElapsedTimeSeconds() > 2) {
//                        intake.allTheWay();
//                    }
//                    if(actionTimer.getElapsedTimeSeconds() > 2.5) {
//                        intake.transferOff();
//                        setPathState(2);
//                    }
//                }
//                else{
//                    intake.intakeOff();
//                }
//                break;
//            case 2:
//                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(paths.collectToShoot());
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(shootGrab2,0.4, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(grabShoot2,0.8, true);
//                    setPathState(-1);
//                }
//                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.resetTimer();
    }
    public int nextPath(int pState) {
        actionTimer.resetTimer();
        return pState+1;
    }

    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);
        follower.setStartingPose(paths.startPose);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        drivetrain = new DrivetrainAuto(hardwareMap, telemetry);
    }

    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    public void start() {
        runtime.reset();
        pathTimer.resetTimer();
        setPathState(0);
    }
}
