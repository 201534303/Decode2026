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
    private double hoodHeight = 0.8;//0.4;//
    private int targetV = 1300;
    private double x = 0.0;

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private int spikeMark = 0;
    public enum PathState {
       START, TO_SHOOT, SHOOT, INTAKE, PARK, TEST
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
            case 2: return paths.shootToSelfee();
            case 3: return paths.shootToSelfee();
            case 4: return paths.shootTo2();
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

            case TO_SHOOT:
                if(!follower.isBusy()){
                    if (intake.haveBall() || waitSecs(6) ) {
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    }
                }
                break;

            case SHOOT:
                shooter.rotateTurret(turnTableAngle);
                shooter.setHood(hoodHeight);

                if(spikeMark == 0 && waitSecs(0.5)){
                    intake.allTheWay();
                    shooter.setHood(hoodHeight);

                    if(waitSecs(2.5)) {
                        resetActionTimer();
                        hoodHeight = 0.4;
                        pathState = PathState.INTAKE;
                    } else if(waitSecs(2)) {
                        hoodHeight = 0.5;
                    } else if(waitSecs(1.5)) {
                        hoodHeight = 0.6;
                    } else if(waitSecs(1)) {
                        hoodHeight = 0.7;
                    }
                } else if(!follower.isBusy()) {
                    if(waitSecs(1.5)) {
                        intake.allTheWay();//run intake + transfer all the way to shoot
                        resetActionTimer();
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                if (!follower.isBusy()  && waitSecs(0.75) && !intake.haveBall()){
                    spikeMark += 1;
                    if (spikeMark == 5) {
                        follower.followPath(paths.shootToPark(), 0.8, true);
                        resetActionTimer();
                        pathState = PathState.PARK;
                        break;
                    }

                    PathChain collectPath = getCollectPath(spikeMark);//gets spike mark path
                    intake.transferOff();
                    intake.intakeIn();
                    follower.followPath(collectPath, 0.8, true);
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
