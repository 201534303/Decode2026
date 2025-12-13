package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
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
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.intakeIn();
                follower.followPath(paths.startToShoot(), 0.6, true);
                setPathState(1);

                break;

            case 1:
//                if (!follower.isBusy() && paths.inBetween(80, 96, 74, 90)) {
//                    telemetry.addData("IT IS: ", "BETWEEN");
//                    intake.allTheWay();
//                }
                if(!follower.isBusy()) { intake.allTheWay(); }
                if(waitSecs(4)){ setPathState(2); }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.transferOff();
                    //intake.intakeIn();
                    follower.followPath(paths.shootToCollect(paths.ballCollect1), 0.6, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.reset(), 0.6, true);
                }
                if(waitSecs(3)){ setPathState(4); }
                break;

            case 4:
                if(!follower.isBusy()) {
                    intake.transferOff();
                    follower.followPath(paths.collectToShoot(), 0.6, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    if(waitSecs(1)) {
                        intake.transferOff();
                        follower.followPath(paths.shootTo2(), 0.6, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(paths.collectToShoot(), 0.6, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    if(waitSecs(1)) {
                        intake.transferOff();
                        follower.followPath(paths.shootTo3(), 0.6, true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(paths.collectToShoot(), 0.6, true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    setPathState(12);
                }
                break;

            case 12:
                if(!follower.isBusy()){
                    if(waitSecs(1)) {
                        intake.transferOff();
                        follower.followPath(paths.shootToOut(), 0.6, true);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOff();
                    shooter.rotateTurret(0);
                }
                break;
        }
    }

    public void setUp(){
        shooter.rotateTurret(-50);
        shooter.hoodPitch(0.75);
    }

    public boolean waitSecs(double seconds){
        return actionTimer.getElapsedTimeSeconds() > seconds;
    }

    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.resetTimer();
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

        setUp();
    }

    public void loop() {
        shooter.close();
        follower.update();

        autonomousPathUpdate();//main auto code

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
