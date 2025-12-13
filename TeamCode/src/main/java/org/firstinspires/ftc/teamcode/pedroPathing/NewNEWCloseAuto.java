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
public class NewNEWCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private ClosePaths paths;

    //robot stuff
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private ElapsedTime runtime = new ElapsedTime();

    private int spikeMark = 0;
    public enum PathState {
        COLLECT_SHOOT, SHOOT_COLLECT, SHOOT,
        OFF, RESET, START
    }
    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                intake.intakeIn();
                follower.followPath(paths.startToShoot(), 0.6, true);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    intake.allTheWay();//go all the way to shoot
                    resetActionTimer();
                    pathState = PathState.SHOOT_COLLECT;
                }
                break;

            case SHOOT_COLLECT:
                if (!follower.isBusy() && waitSecs(1.0)) {//waits 1 sec
                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark pos
                    if (collectPath != null) {//if there is a spike pos
                        intake.intakeIn();
                        intake.transferOff();
                        follower.followPath(collectPath, 0.6, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case COLLECT_SHOOT:
                if (!follower.isBusy()) {
                    if (spikeMark == 0) {
                        spikeMark--;
                        pathState = PathState.RESET;
                    }
                    else if (spikeMark < 3){
                        if (spikeMark < 0){ spikeMark = 0; }
                        follower.followPath(paths.collectToShoot(), 0.6, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case RESET:
                if(!follower.isBusy()) {
                    follower.followPath(paths.reset(), 0.6, true);
                }
                if(waitSecs(3)){
                    resetActionTimer();
                    pathState = PathState.COLLECT_SHOOT;
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shootToOut(), 0.6, true);
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOff();
                    shooter.rotateTurret(0);
                }
                break;
        }
    }

    private PathChain getCollectPath(int spikeMark) {
        switch (spikeMark) {
            case 0: return paths.shootTo1();
            case 1: return paths.shootTo2();
            case 2: return paths.shootTo3();
            default: return null;
        }
    }

    public void setUp(){
        shooter.rotateTurret(-50);
        shooter.hoodPitch(0.87);
    }

    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);
        follower.setStartingPose(paths.startPose);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        setUp();
    }

    public void loop() {
        shooter.close();
        follower.update();

        autonomousPathUpdate();//main auto code

        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    public void start() {
        runtime.reset();
        pathTimer.resetTimer();
        pathState = PathState.START;
    }
}
