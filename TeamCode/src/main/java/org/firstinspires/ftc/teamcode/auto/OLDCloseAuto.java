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

    private int spikeMark = 1;
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
                    if(spikeMark == 3) {
                        if(waitSecs(2)) {
                            resetActionTimer();
                            pathState = PathState.INTAKE;
                        }
                    }
                    else {
                        if(waitSecs(0.75)) {
                            resetActionTimer();
                            pathState = PathState.INTAKE;
                        }
                    }
                }
                break;

            case INTAKE:
                if (!follower.isBusy()) {//waits 0.5 works
                    intake.transferOff();

                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark pos

                    if (spikeMark < 4) {//if there is a spike pos
                        follower.followPath(collectPath, 0.7, true);
                        if(spikeMark == 1){
                            reset = true;
                        }
                        if(follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark < 5) {
                        follower.followPath(collectPath, 0.9, true);

                        if(follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.UP;
                        }
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case TO_SHOOT:
                if(spikeMark != 0) {
                    shooter.rotateTurret(47.5);
                }

                if (!follower.isBusy()) {
                    if(spikeMark == 0){
                        follower.followPath(paths.startToShoot(), 0.7, true);
                        if(follower.atParametricEnd()){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    }

                    intake.setIntakeSpeed(0.5);
                    if (reset) {
                        follower.followPath(paths.reset(), 0.75, false);
                        if(follower.atParametricEnd() && waitSecs(0.75)) {
                            reset = false;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark < 4){
                        if(!once) {
                            follower.followPath(paths.collectToShoot(), 0.8, true);
                            once = true;
                        }
                        if(follower.atParametricEnd()) {
                            once = false;
                            spikeMark++;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark < 5) {
                        if(!once) {
                            follower.followPath(paths.collectToShoot(), 0.9, true);
                            once = true;
                        }
                        if(follower.atParametricEnd()) {
                            once = false;
                            spikeMark++;
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else {
                        follower.followPath(paths.shootToOut(), 0.7, true);

                        if(follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.OFF;
                        }
                    }
                }
                break;

            case UP:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootTo4(), 0.6, true);

                    if (follower.atParametricEnd() || waitSecs(2)) {
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
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
            case 1: return paths.shootTo1();
            case 2: return paths.shootTo2();
            case 3: return paths.shootTo3();
            case 4: return paths.shootTo4Mid();
            default: return null;
        }
    }

    public void setUp(){
        shooter.setHood(0.2);
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
        shooter.close();
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
