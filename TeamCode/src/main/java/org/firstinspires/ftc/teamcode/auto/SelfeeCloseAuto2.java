package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "SelfeeCloseAuto")

public class SelfeeCloseAuto2 extends OpMode {
    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private double turnTableAngle = 45;
    private double turnTableAngleFirst = 13;
    private double hoodHeight = 0.30;//0.4;//
    private int targetV = 1200;
    private double x = 0.0;

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private int spikeMark = 0;
    public enum PathState {
       START, TO_SHOOT, SHOOT, INTAKE, PARK, FIRST_SHOOT,
        OUT, IN
    }
    PathState pathState = PathState.START;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMirror = false;
    private boolean readyAlliance = false;
    private boolean done = false;
    private boolean ready = false;
    private boolean ran = false;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                resetActionTimer();
                pathState = PathState.FIRST_SHOOT;
                break;

            case FIRST_SHOOT:
                if (waitSecs(0.75) && !ready) {
                    ready = true;
                }
                if (waitSecs(0.25) && ready) {
                    intake.allTheWay();
                    targetV += 500;
                    resetActionTimer();
                }

                if(!follower.isBusy()) {
                    follower.followPath(paths.fistToShoot(), 0.9, true);

                    if (follower.atParametricEnd()) {
                        resetActionTimer();
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case TO_SHOOT:
                shooter.rotateTurret(turnTableAngle);
                shooter.setHood(hoodHeight);

                if(!follower.isBusy()) {
                    if (spikeMark == 1) {
                        follower.followPath(paths.ballCollect1ToShoot(), 0.9, true);
                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 2 || spikeMark == 3 /*|| spikeMark == 4*/) {
                        follower.followPath(paths.selfeeToShoot(), 0.9, true);
                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if(spikeMark == 4){
                        follower.followPath(paths._2ToShoot(), 0.9, true);
                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    }
                }
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    if (waitSecs(1.25)) {
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                if(intake.haveBall()){
                    resetActionTimer();
                    spikeMark += 1;
                    pathState = PathState.TO_SHOOT;
                }

                if (!follower.isBusy()) {
                    intake.transferOff();
                    intake.intakeIn();

                    if (spikeMark == 0) {
                        follower.followPath(paths.shootTo1(), 0.9, true);

                        if (follower.atParametricEnd() && waitSecs(3)) {
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark == 1 || spikeMark == 2 /*|| spikeMark == 3*/){
                        follower.followPath(paths.shootToSelfee(), 0.9, true);

                        if (follower.atParametricEnd() && waitSecs(6) || waitSecs(8)) {
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                            //pathState = PathState.OUT;
                        }
                    } else if (spikeMark == 3){
                        follower.followPath(paths.shootTo2(), 0.9, true);

                        if (follower.atParametricEnd()) {
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark == 4){
                        follower.followPath(paths.shootToPark(), 0.9, true);

                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            pathState = PathState.PARK;
                        }
                    }
                }
                break;

            case OUT:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                } else if (!follower.isBusy()){
                    follower.followPath(paths.selfeeWiggle1(), 0.8, true);

                    if(follower.atParametricEnd()) {
                        resetActionTimer();
                        pathState = PathState.IN;
                    }
                }
                break;

            case IN:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                } else if (!follower.isBusy()){
                    follower.followPath(paths.selfeeWiggle2(), 0.8, true);
                    if(follower.atParametricEnd() || waitSecs(3)) {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case PARK:
                if(!follower.isBusy()) {
                    done = true;
                    shooter.rotateTurret(0);
                    intake.off();
                }
                break;
        }
    }

    public void init() {
        actionTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        shooter.setHood(0.80);
    }

    public void init_loop(){
        readyAlliance = choose.allianceInit();
        alliance = choose.getSelectedAlliance();
        telemetry.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);

        isMirror = paths.bluePath(alliance);//mirrors the paths if blue
        follower.setStartingPose(paths.startPose);//sets up the starting pose

        if(isMirror) {
            turnTableAngleFirst = -13;
            turnTableAngle = -45;
        }//if it's mirrored turn the turntable
        shooter.rotateTurret(turnTableAngleFirst);

        runtime.reset();//resets overall timer
        pathState = PathState.START;//sets the path state
    }

    public void loop() {
        if (spikeMark == 0){
            shooter.closeMove(targetV);
        } else if (!done) {
            shooter.close();
        }
        follower.update();

        autonomousPathUpdate();//main auto code

        //telemetry.addData("haveBall", intake.haveBall());
        telemetry.addData("mirror", isMirror);
        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("alliance", alliance);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }
}
