package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "CloseAuto")

public class CloseAuto extends OpMode {
    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private double turnTableAngle = 47.5;
    private double hoodHeight = 0.2;//0.4;//

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private Choose choose;
    private Timer actionTimer;
    private int spikeMark = 1;
    private int maxTrips = 4;
    public enum PathState {
        COLLECT_SHOOT, SHOOT_COLLECT, SHOOT,
        OFF, RESET, START, UP
    }
    PathState pathState = PathState.START;
    private Choose.Alliance alliance = Choose.Alliance.RED;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMirror = false;
    private boolean readyTrips = false;
    private boolean readyAlliance = false;
    private boolean readyWolfpack = false;
    private boolean done = false;
    private boolean wolfpack = false;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                follower.followPath(paths.startToShoot(), 0.8, true);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    shooter.rotateTurret(turnTableAngle);
                    shooter.setHood(0.48);

                    if(spikeMark == 3 || spikeMark == 2 || spikeMark == 1) {//if on the 3rd one
                        if(waitSecs(1.5)) {//waits 2
                            intake.allTheWay();//run intake + transfer all the way to shoot
                            resetActionTimer();
                            pathState = PathState.SHOOT_COLLECT;
                        }
                    } else {
                        intake.allTheWay();//go all the way to shoot
                        resetActionTimer();
                        pathState = PathState.SHOOT_COLLECT;
                    }
                }
                break;

            case SHOOT_COLLECT:
                if (!follower.isBusy() && waitSecs(0.75)) {//waits to finish shooting
                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark path
                    if (spikeMark <= maxTrips && spikeMark < 4) {
                        intake.intakeIn();
                        intake.transferOff();
                        follower.followPath(collectPath, 0.8, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    } else if (spikeMark <= maxTrips && spikeMark < 5) {
                        intake.intakeIn();
                        intake.transferOff();
                        follower.followPath(collectPath, 0.9, true);
                        resetActionTimer();
                        pathState = PathState.UP;
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case COLLECT_SHOOT:
                if (!follower.isBusy() &&  waitSecs(1) || waitSecs(3)) {
                    if (spikeMark == 1) {//for 1st one
                        intake.setIntakeSpeed(0.7);
                        spikeMark = 0;
                        follower.followPath(paths.reset(), 0.8, false);
                        resetActionTimer();
                        pathState = PathState.RESET;
                    } else if (spikeMark == 2 && wolfpack) {//for wolfpack auto
                        spikeMark = -1;
                        follower.followPath(paths.reset2(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.RESET;
                    }
                    else if (spikeMark <= maxTrips && spikeMark < 4){
                        intake.setIntakeSpeed(0.7);
                        if (spikeMark == 0){ spikeMark = 1; }
                        else if (spikeMark == -1){ spikeMark = 2; }
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else if (spikeMark <= 5) {
                        intake.setIntakeSpeed(0.7);
                        waitSecs(1);
                        follower.followPath(paths.collectToShoot(), 0.9, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else { pathState = PathState.OFF; }
                }
                break;

            case RESET:
                if(!follower.isBusy() || waitSecs(3)/*|| paths.inBetween(110, 120,70,80)*/) {
                    if(waitSecs(2.5) && !wolfpack){
                        pathState = PathState.COLLECT_SHOOT;
                    } else if(waitSecs(3) && wolfpack){//wait more for wolfpack reset
                        pathState = PathState.COLLECT_SHOOT;
                    }
                }

                break;

            case UP:
                if (!follower.isBusy() /*|| waitSecs(13)*/) {
                    follower.followPath(paths.shootTo4(), 0.7, true);
//                    if(waitSecs(1)){
//                        intake.setIntakePower(-0.7);
//                    }
                    if (!follower.isBusy() || waitSecs(2)) {
                        intake.intakeIn();
                        pathState = PathState.COLLECT_SHOOT;
                    }
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    done = true;
                    if (maxTrips > 1) {
                        follower.followPath(paths.shootToOut(), 0.8, true);
                    } else{
                        follower.followPath(paths.shootToOutDown(), 0.8, true);
                    }
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

    public void init() {
        actionTimer = new Timer();

        choose = new Choose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        shooter.setHood(0.48);
    }

    public void init_loop(){
        telemetry.addData("servoPos", 1-shooter.servoPos());
        shooter.setHood(0.48);

        if(!readyWolfpack){
            readyWolfpack = choose.wolfpackInit();
            wolfpack = choose.getSelectedWolfpack();
            if (wolfpack){ maxTrips = 2; }
        } else if (!readyTrips && !wolfpack){//run trips selection only if wolfpack false
            readyTrips = choose.tripsInit();
            maxTrips = choose.getMark();
        } else if(!readyAlliance) { //run alliance selection
            readyAlliance = choose.allianceInit();
            alliance = choose.getSelectedAlliance();
        } else{
            choose.displayReadyCloseScreen();
        }

        telemetry.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);

        isMirror = paths.bluePath(alliance);//mirrors the paths if blue
        follower.setStartingPose(paths.startPose);//sets up the starting pose

        if(isMirror) {turnTableAngle = -43; }//if it's mirrored turn the turntable
        shooter.rotateTurret(turnTableAngle);//rotates the turntable

        runtime.reset();//resets overall timer
        actionTimer.resetTimer();//resets path timer
        pathState = PathState.START;//sets the path state
    }

    public void loop() {
        if (!done) { shooter.close(); }
        follower.update();

        autonomousPathUpdate();//main auto code

        telemetry.addData("servoPos", 1-shooter.servoPos());
        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("wolfpack", wolfpack);
        telemetry.addData("max trips", maxTrips);
        telemetry.addData("alliance", alliance);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }
    public void stop(){

    }
}
