package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CloseAuto_12.PathState.OFF;

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

@Autonomous(name = "12 Close")

public class CloseAuto_12 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private ClosePaths paths;
    double loopCounter;

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
                //intake.setIntakeSpeed(0.4);
                follower.followPath(paths.startToShoot(), 0.7, true);
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
                if (!follower.isBusy() && waitSecs(0.6)) {//waits 0.5 works
                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark pos
                    if (spikeMark < 3) {//if there is a spike pos
                        intake.intakeIn();
                        intake.transferOff();
                        follower.followPath(collectPath, 0.65, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    }
                    else if (spikeMark > 2 && spikeMark < 5){
                        intake.intakeIn();
                        intake.transferOff();
                        follower.followPath(collectPath, 1, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    }
                    else { pathState = OFF; }
                }
                break;

            case COLLECT_SHOOT:
                if (!follower.isBusy()) {
                    if (spikeMark == 0) {
                        spikeMark--;
                        pathState = PathState.RESET;
                    }
                    else if (spikeMark < 5){
                        //intake.setIntakeSpeed(0);
                        if (spikeMark < 0){ spikeMark = 0; }
                        //test a BezierCurve instead of Linear
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    }
                    else { pathState = OFF; }
                }
                break;

            case RESET:
                //intake.intakeIn();
                if(!follower.isBusy()) {
                    follower.followPath(paths.reset(), 0.7, true);
                }
                if(waitSecs(1.5)){//used to be 1.5
                    resetActionTimer();
                    pathState = PathState.COLLECT_SHOOT;
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shootToOut(), 0.7, true);
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOff();
                    //shooter.setTurretpos(0);
                }
                break;
        }
    }

    private PathChain getCollectPath(int spikeMark) {
        switch (spikeMark) {
            case 0: return paths.shootTo1();
            case 1: return paths.shootTo2();
            case 2: return paths.shootTo3();
            case 3: return paths.shootTo4();
            case 4: return paths.shootTo5();
            default: return null;
        }
    }

    public void setUp(){
        shooter.hoodPitch(0.5);
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
    public void init_loop(){
        //shooter.setTurretpos(1);
    }

    public void loop() {
        shooter.close();
        follower.update();

        if (pathState != OFF && loopCounter == 20){
            //shooter.setTurretpos(1);
            loopCounter = 0;
        } else {
            loopCounter += 1;
            if(!follower.isBusy()) {
                //shooter.setTurretpos(0);
            }
        }

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
