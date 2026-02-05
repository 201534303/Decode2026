package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "SelfeeCloseAuto")

public class SelfeeCloseAuto extends OpMode {
    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private double turnTableAngle = 47.5;
    private double hoodHeight = 0.6;//0.4;//
    private int targetV = 1300;
    private double x = 0.0;

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private int spikeMark = 0;
    public enum PathState {
       START, TO_SHOOT, SHOOT, INTAKE, PARK, TEST,
        OUT, IN
    }
    PathState pathState = PathState.START;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMirror = false;
    private boolean readyAlliance = false;
    private boolean done = false;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    private  PathChain getCollectPath(int spikeMark){
        switch (spikeMark) {
            case 1: return paths.shootTo1();
            case 2:
            case 3:
            case 4:
                return paths.shootToSelfee();
            case 5: return paths.shootTo2();
            default: return null;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(paths.fistToShoot(), 0.8, true);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case TEST:
                intake.intakeIn();
                break;

            case TO_SHOOT:
                if(!follower.isBusy() || waitSecs(3)){
                    if(spikeMark == 5){
                        if (intake.haveBall() || waitSecs(1)){
                            follower.followPath(paths.collectToShoot(), 0.9, true);
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 1){
                        if (intake.haveBall() || waitSecs(2) ) {
                            follower.followPath(paths.ballCollect1ToShoot(), 0.9, true);
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4) {
                        if (intake.haveBall() || waitSecs(3) ) {
                            follower.followPath(paths.collectToShoot(), 0.9, true);
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    }

                }
                break;

            case SHOOT:
                shooter.rotateTurret(turnTableAngle);
                shooter.setHood(hoodHeight);

                if(!follower.isBusy() && waitSecs(1)) {
                    intake.allTheWay();//run intake + transfer all the way to shoot
                    resetActionTimer();
                    pathState = PathState.INTAKE;
                }
                break;

            case INTAKE:
                if (!follower.isBusy()  && waitSecs(0.75) ){
                    intake.transferOff();
                    intake.intakeIn();

                    spikeMark += 1;
                    if (spikeMark == 6) {
                        //follower.followPath(paths.shootToPark(), 0.8, true);
                        follower.followPath(paths.toStart(), 0.8, true);

                        resetActionTimer();
                        pathState = PathState.PARK;
                        break;
                    }

                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark path
                    follower.followPath(collectPath, 0.7, true);

                    if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4){
                        resetActionTimer();
                        //pathState = PathState.TO_SHOOT;
                        pathState = PathState.OUT;
                    } else {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case OUT:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                } else if (!follower.isBusy() && waitSecs(3)){
                    follower.followPath(paths.selfeeWiggle1(), 0.7, true);
                    resetActionTimer();
                    pathState = PathState.IN;
                }
                break;

            case IN:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                } else if (!follower.isBusy() && waitSecs(1)){
                    follower.followPath(paths.selfeeWiggle2(), 0.7, true);
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
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

        shooter.setHood(hoodHeight);
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

        if(isMirror) {turnTableAngle = -43; }//if it's mirrored turn the turntable
        shooter.rotateTurret(turnTableAngle);//rotates the turntable

        runtime.reset();//resets overall timer
        //actionTimer.resetTimer();//resets path timer
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

        telemetry.addData("actionTimer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("servoPos", 1-shooter.servoPos());
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

    public void stop(){

    }
}
