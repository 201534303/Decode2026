package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "CloseAuto")

public class CloseAuto extends OpMode {
    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private double turnTableAngle = 47.5;

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

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                follower.followPath(paths.startToShoot(), 0.7, true);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case SHOOT:

                //on way to shoot
                if(spikeMark >= 5 && follower.getPose().getY() < 12){
                    intake.setIntakeSpeed(-1);
                }
                else{
                    intake.setIntakeSpeed(0.7);
                }

                if(!follower.isBusy()) {
                    shooter.rotateTurret(turnTableAngle);

                    if(spikeMark == 3 || spikeMark == 2 || spikeMark == 1) {//if on the 3rd one
                        if(waitSecs(1.5)) {//waits 2
                            intake.allTheWay();//run intake + transfer all the way to shoot
                            resetActionTimer();
                            pathState = PathState.SHOOT_COLLECT;
                        }
                    }
                    else {
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
                        follower.followPath(collectPath, 0.7, true);
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
                if (!follower.isBusy() &&  waitSecs(1)) {
                    if (spikeMark == 1) {//for 1st one
                        intake.setIntakeSpeed(0.7);
                        spikeMark--;
                        follower.followPath(paths.reset(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.RESET;
                    } /*else if (spikeMark == 1) {//for wolfpack auto
                        spikeMark--;
                        follower.followPath(paths.reset(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.RESET;
                    }*/
                    else if (spikeMark <= maxTrips && spikeMark < 4){
                        intake.setIntakeSpeed(0.7);
                        if (spikeMark < 1){ spikeMark = 1; }
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else if (spikeMark <= 5) {
                        intake.setIntakeSpeed(0.7);
                        follower.followPath(paths.collectToShoot(), 0.9, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else { pathState = PathState.OFF; }
                }
                break;

            case RESET:
                if(!follower.isBusy() /*|| paths.inBetween(110, 120,70,80)*/) {
                    if(waitSecs(2.5)){//0.75
                        pathState = PathState.COLLECT_SHOOT;
                    }
                }

                break;

            case UP:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootTo4(), 0.6, true);
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
                    follower.followPath(paths.shootToOut(), 0.7, true);
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

        shooter.setHood(0.3);
    }

    public void init_loop(){
        if (!readyTrips){//run trips selection
            readyTrips = choose.tripsInit();
            maxTrips = choose.getMark();
        } else{ //run alliance selection
            choose.allianceInit();
            alliance = choose.getSelectedAlliance();
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
}
