package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.SelfeeCloseAuto2.PathState.PARK;
import static org.firstinspires.ftc.teamcode.auto.SelfeeCloseAuto2.PathState.TO_PARK;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

@Autonomous(name = "SelfeeCloseAuto")

public class SelfeeCloseAuto2 extends OpMode {
    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private RobotActions robotActions;
    private double turnTableAngle = 47;
    private double turnTableAngleFirst = 13;
    private double hoodHeight = 0.44498;//0.4;//
    private int targetV = 1400;
    private double x = 0.0;

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private Timer overallTimer;
    private int spikeMark = 0;
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK, FIRST_SHOOT, TO_PARK
    }
    PathState pathState = PathState.START;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private boolean fill = false;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMirror = false;
    private boolean readyAlliance = false;
    private boolean readyFill = false;
    private boolean parkActions = false;
    private boolean done = false;
    private boolean ready = false;
    private boolean ran = false;
    private boolean doneOne = false;
    double movingHood = 0;
    private boolean firstShootPathSet = false;
    private boolean toShootPathSet = false;
    private boolean intakePathSet = false;
    private boolean didParking = false;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                resetActionTimer();
                pathState = PathState.FIRST_SHOOT;
                break;

            case FIRST_SHOOT:
//                if (waitSecs(0.75) && !ready) {
//                    ready = true;
//                }
//                if (waitSecs(0.25) && ready) {
//                    intake.allTheWay();
//                    //shooter.setHood(0.8-movingHood);
//                    movingHood += 0.05;
//                    targetV += 50;
//                    resetActionTimer();
//                }

                if(!follower.isBusy() /*&& !firstShootPathSet*/) {
                    firstShootPathSet = true;
                    follower.followPath(paths.firstToShoot(), 1, false);
                }

                if (follower.atParametricEnd() && firstShootPathSet) {
                    resetActionTimer();
                    doneOne = true;
                    firstShootPathSet = false;
                    pathState = PathState.SHOOT;
                }
                break;

            case TO_SHOOT:
                //shooter.rotateTurret(turnTableAngle);
                shooter.setHood(hoodHeight);

                if (!follower.isBusy() && !toShootPathSet) {
                    toShootPathSet = true;
                    if (spikeMark == 1) {
                        follower.followPath(paths.ballCollect1ToShoot(), 1, true);
                    } else if (spikeMark == 2 || (spikeMark == 4 && !fill) || spikeMark == 3) {
                        follower.followPath(paths.selfeeToShoot(), 1, true);
                    } else if(spikeMark == 4){
                        follower.followPath(paths._2ToShoot(), 1, true);
                    } else if (spikeMark == 5 && !fill) {
                        follower.followPath(paths._2ToShoot2(), 1, true);
                    } else if (spikeMark == 5) {
                        follower.followPath(paths._3ToShoot(), 1, true);
                    }
                }
                if (toShootPathSet) {
                    if ((spikeMark == 2 || spikeMark == 3 || spikeMark == 4 || spikeMark == 5) && waitSecs(0.75)) {
                        intake.setIntakeSpeed(0.3);
                    }
                    if (follower.atParametricEnd()) {
                        toShootPathSet = false;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    }
                }
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    if (waitSecs(0.6)) {//0.6
                        resetActionTimer();
                        intakePathSet = false;
                        toShootPathSet = false;
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                if(intake.haveBall() && waitSecs(0.75)){
                    resetActionTimer();
                    spikeMark += 1;
                    intakePathSet = false;
                    pathState = PathState.TO_SHOOT;
                    break;
                }

                if (!follower.isBusy()) {
                    intake.transferOff();
                    intake.intakeIn();

                    if (!intakePathSet) {
                        intakePathSet = true;
                        if (spikeMark == 0) {
                            follower.followPath(paths.shootTo1(), 1, true);
                        } else if (spikeMark == 1 || spikeMark == 2 ) {
                            follower.followPath(paths.shootToSelfee(), 1, true);
                        } else if (!fill && spikeMark == 3){
                            follower.followPath(paths.shootToSelfee2(), 1, true);
                        } else if ( (spikeMark == 4 && !fill) || (spikeMark == 3 && fill)) {
                            follower.followPath(paths.shootTo2(), 1, true);
                        } else if (fill && spikeMark == 4) {
                            follower.followPath(paths.shootTo3(), 1, true);
                        } else if (spikeMark == 5) {
                            follower.followPath(paths.shootToPark(), 0.6, true);
                        }
                    }

                    if (spikeMark == 0 && (follower.atParametricEnd() && waitSecs(1) || waitSecs(2))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if ((spikeMark == 1 || (spikeMark == 3 && !fill) || spikeMark == 2) && (follower.atParametricEnd() && waitSecs(3.75) || waitSecs(4))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (((spikeMark == 4 && !fill) || (spikeMark == 3 && fill )) && (follower.atParametricEnd() && waitSecs(0.5) || waitSecs(1))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if ( (spikeMark == 4 && fill) && (follower.atParametricEnd() && waitSecs(3.5) || waitSecs(4))){
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                    else if (spikeMark == 5 && follower.atParametricEnd()) {
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.PARK;
                    }
                }
                break;

            case TO_PARK:
                intake.off();
                done = true;

                if (!didParking) {
                    follower.followPath(paths._ToPark(), 0.6, true);
                    didParking = true;
                    pathState = PARK;
                }
                break;

            case PARK:
                if(!parkActions) {
                    parkActions = true;
                    done = true;
                    shooter.rotateTurret(0);
                    intake.transferOff();
                    intake.off();
                    shooter.off();
                }
                break;
        }
    }

    public void init() {
        actionTimer = new Timer();
        overallTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        robotActions = new RobotActions(shooter, follower, telemetry);

        shooter.setHood(0.60);
    }

    public void init_loop(){
        if(!readyAlliance) {
            alliance = choose.getSelectedAlliance();
            readyAlliance = choose.allianceInit();
        } else {
            fill = choose.getFill();
            readyFill = choose.fillInit();
        }

        if(alliance == OLDChoose.Alliance.BLUE){
            isMirror = true;
        } else {
            isMirror = false;
        }

        if (isMirror) {
            turnTableAngleFirst = -5;
            turnTableAngle = -45;
        } else {
            turnTableAngle = 47;
            turnTableAngleFirst = 13;
        }

        shooter.rotateTurret(turnTableAngleFirst);

        telemetry.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);

        isMirror = paths.bluePath(alliance);//mirrors the paths if blue
        follower.setStartingPose(paths.startPose);//sets up the starting pose

        if(isMirror) {
            turnTableAngleFirst = -9;
            turnTableAngle = -45;
        }//if it's mirrored turn the turntable

        shooter.rotateTurret(turnTableAngleFirst);

        runtime.reset();//resets overall timer
        overallTimer.resetTimer();
        actionTimer.resetTimer();
        pathState = PathState.START;//sets the path state
    }

    public void loop() {
        /*if (spikeMark == 0 && !doneOne){
            shooter.closeMove(targetV);
        } else */if (!done  /*&& doneOne*/) {
            robotActions.updateTurret(alliance, follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            shooter.close();
        }
        follower.update();

        autonomousPathUpdate();//main auto code

        //telemetry.addData("haveBall", intake.haveBall());
        telemetry.addData("mirror", isMirror);
        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("alliance", alliance);
        telemetry.addData("runtime", overallTimer);

//        if(overallTimer.getElapsedTimeSeconds() > 29){
//            pathState = TO_PARK;
//        }

        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());
        //telemetry.addData("heading", follower.getPose().getHeading());
        //telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    @Override
    public void stop(){
        for (int i  = 0; i < 50; i++){
            Pose p = follower.getPose();
            PoseSaver.save(p.getX(), p.getY(), p.getHeading());
            follower.update();
        }
    }
}
