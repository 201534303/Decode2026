package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDClosePaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "OldCloseAuto")
//Disabled
public class OLDCloseAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private OLDClosePaths paths;

    //robot stuff
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private OLDChoose choose;
    private ElapsedTime runtime = new ElapsedTime();

    private int spikeMark = 0;
    private int maxTrips = 4;
    private boolean reset = false;
    private boolean once = false;
    public enum PathState {
        INTAKE, TO_SHOOT, SHOOT,
        OFF, RESET, START, UP
    }
    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                resetActionTimer();
                pathState = PathState.TO_SHOOT;
                break;

            case SHOOT:
                intake.allTheWay();

                if(!follower.isBusy()) {
                    if(waitSecs(0.75)) {
                        resetActionTimer();
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                if (!follower.isBusy()) {//waits 0.5 works
                    intake.intakeIn();
                    intake.transferOffOff();

                    if(spikeMark == 1){
                        follower.followPath(paths.shootTo1(), 0.8, true);

                        if(follower.atParametricEnd() && waitSecs(0.5)) {
                            resetActionTimer();
                            pathState = PathState.RESET;
                        }
                    } else if(spikeMark == 2){
                        follower.followPath(paths.shootTo2(), 0.8, true);

                        if(follower.atParametricEnd() && waitSecs(0.5)) {
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if(spikeMark == 3){
                        follower.followPath(paths.shootTo3(), 0.8, true);

                        if(follower.atParametricEnd() && waitSecs(0.5)) {
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if(spikeMark == 4){
                        follower.followPath(paths.shootTo4Mid(), 0.8, true);

                        if(follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.UP;
                        }
                    } else if(spikeMark == 5){
                        follower.followPath(paths.shootToOut(), 0.9, true);
                        if(follower.atParametricEnd()){
                            resetActionTimer();
                            pathState = PathState.OFF;
                        }
                    }
                }
                break;

            case RESET:
                if(!once) {
                    follower.followPath(paths.reset(), 0.7, true);
                    once = true;
                }

                if(follower.atParametricEnd()) {
                    if(waitSecs(2.5)){
                        once = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;
            case TO_SHOOT:
                if (!follower.isBusy()) {
                    if(spikeMark != 0) {
                        shooter.rotateTurret(47.5);
                    }

                    if(spikeMark == 0){
                        follower.followPath(paths.startToShoot(), 0.8, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if(spikeMark == 1){
                        follower.followPath(paths.resetToShoot(), 0.8, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if(spikeMark == 2){
                        follower.followPath(paths.twoToShoot(), 0.8, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if(spikeMark == 3){
                        follower.followPath(paths.threeToShoot(), 0.8, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if(spikeMark == 4){
                        follower.followPath(paths.fourToShoot(), 1, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    }
                }
                break;

            case UP:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootTo4(), 0.8, true);

                    if (follower.atParametricEnd() || waitSecs(1.50)) {
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOffOff();
                    shooter.rotateTurret(0);
                }
                break;
        }
    }

    private PathChain getCollectPath(int spikeMark) {
        switch (spikeMark) {
            case 1: return paths.shootTo1();
            case 2: return paths.shootTo2();
            case 3: return paths.shootTo3();
            case 4: return paths.shootTo4Mid();
            default: return null;
        }
    }

    public void setUp(){
        shooter.setHood(0.3);
        shooter.rotateTurret(47.5);
    }

    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        follower = Constants.createFollower(hardwareMap);
        paths = new OLDClosePaths(follower);
        follower.setStartingPose(paths.startPose);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        setUp();
    }

    public void init_loop(){
        boolean ready = choose.tripsInit();
        maxTrips = choose.getMark();

        /*if (ready){
            maxTrips = choose.getMark();
        }*/
        telemetry.update();
    }

    public void loop() {
        shooter.closeSlow();
        follower.update();

        autonomousPathUpdate();//main auto code

        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("max trips", maxTrips);
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
