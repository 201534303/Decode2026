package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "FarAuto")

public class FarAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private FarPaths paths;

    private IntakeAuto intake;
    private ShooterAuto shooter;
    private Choose choose;
    private ElapsedTime runtime = new ElapsedTime();

    private int spikeMark = 1;
    private int maxTrips = 4;
    private boolean once = false;
    public enum PathState {
        COLLECT_SHOOT, SHOOT_COLLECT, SHOOT,
        OFF, START, IN, OUT
    }
    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                resetActionTimer();
                follower.followPath(paths.startToShoot(), 0.75, false);
                pathState = PathState.SHOOT;
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    //if (spikeMark == 1){
                        if (waitSecs(1.8)){
                            shooter.rotateTurret(63);
                            intake.allTheWay();//go all the way to shoot
                            resetActionTimer();
                            pathState = PathState.SHOOT_COLLECT;
                        }
                    /*} else{
                        shooter.rotateTurret(63);
                        intake.allTheWay();//go all the way to shoot
                        resetActionTimer();
                        pathState = PathState.SHOOT_COLLECT;
                    }*/
                }
                break;

            case SHOOT_COLLECT:
                if (!follower.isBusy() && waitSecs(0.75)) {//waits 0.5 works
                    intake.intakeIn();
                    intake.transferOff();

                    if (spikeMark < 4){
                        follower.followPath(paths.shootTo3(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.OUT;
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case COLLECT_SHOOT:
                if (!follower.isBusy()  &&  waitSecs(1) || waitSecs(2)) {
                    intake.transferOff();
                    intake.setIntakeSpeed(0.5);
                    if (spikeMark < 4) {
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else{ pathState = PathState.OFF; }
                }
                break;


            case OUT:
                if(!follower.isBusy() && waitSecs(1.5) || waitSecs(2)){
                    intake.setIntakePower(-0.7);
                    follower.followPath(paths.Out3(), 0.8, true);
                    resetActionTimer();
                    pathState = PathState.IN;
                }
                break;

            case IN:
                if(!follower.isBusy() && waitSecs(1)){
                    intake.setTransferPower(0.2);
                    intake.intakeIn();
                    follower.followPath(paths.In3(), 0.8, true);
                    resetActionTimer();
                    if (!once){
                        pathState = PathState.OUT;
                        once = true;
                    } else {
                        //intake.setTransferPower(0.3);
                        pathState = PathState.COLLECT_SHOOT;
                    }
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shootToOut(), 0.7, true);
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOff();
                    shooter.rotateTurret(0);
                }
                break;
        }
    }

    public void setUp(){
        shooter.setHood(0.5);
        shooter.rotateTurret(63);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        choose = new Choose(gamepad1, telemetry);
        follower = Constants.createFollower(hardwareMap);
        paths = new FarPaths(follower);
        follower.setStartingPose(paths.startPose);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        setUp();
        resetActionTimer();
    }

    public void init_loop(){
//        choose.tripsInit();
//        maxTrips = choose.getMark();
        telemetry.update();
    }

    @Override
    public void loop() {
        shooter.far();
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
