package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.BallDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "FarAuto")

public class FarAutoAttempt2 extends OpMode {
    // Robot Subsystems
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private BallDetection limelight;

    // Timers
    private Timer actionTimer;
    private ElapsedTime runtime = new ElapsedTime();

    // Choose Auto Type
    private OLDChoose choose;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private boolean isMirror = false;

    // Folower
    private Follower follower;
    private FarPaths paths;
    private Telemetry dash;

    // Actions
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK,
        OUT, IN, DETECT
    }
    PathState pathState = PathState.START;

    // Random Variables
    private int spikeMark = 0;
    private boolean done = false;
    private boolean detect = false;
    Pose ballCollect;
    Pose newOut;
    private int count = 0;
    private int shootCount = 0;
    private int count2 = 0;
    private int stuckRun = 0;
    private int inPark = 0;
    private double turnTableAngle = 68;
    private double hoodHeight = 0.225;

    // For Vision
    double x;
    double y;

    // Timer Control
    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    // Main Auto Code
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                shooter.far();
                if (waitSecs(1.25)) { // waits for flywheel to speed up
                    resetActionTimer(); // resets timer
                    pathState = PathState.SHOOT; // sets to shoot state
                }
                break;

            case SHOOT:
                if (!follower.isBusy()) {
                    intake.allTheWaySlow();// go all the way to shoot

                    if (spikeMark == 0) {
                        if (waitSecs(1)) { // waits 1 sec to wait for all balls to shoot
                            resetActionTimer();
                            pathState = PathState.INTAKE;
                        }
                    } else if (spikeMark == 1){
                        if (waitSecs(1.75)) { // waits 1 sec to wait for all balls to shoot
                            resetActionTimer();
                            pathState = PathState.INTAKE;
                        }
                    } else if (spikeMark == 2 || spikeMark == 3) {
                        if (waitSecs(1.75)) { // waits 1 sec to wait for all balls to shoot
                            resetActionTimer();
                            spikeMark += 1;
                            pathState = PathState.DETECT;
                        }
                    } else if (spikeMark == 4) {
                        if (waitSecs(1.75)) { // waits 1 sec to wait for all balls to shoot
                            park();
                            follower.followPath(paths.shootToPark(), 0.8, false);

                            if(follower.atParametricEnd()) {
                                resetActionTimer();
                                pathState = PathState.PARK;
                            }
                        }
                    }
                }
                break;

            case INTAKE:
                if (intake.haveBall()){
                    resetActionTimer();
                    spikeMark += 1;
                    pathState = PathState.TO_SHOOT;
                }

                if (!follower.isBusy()) {
                    intake.transferOff();
                    intake.intakeIn();

                    if (spikeMark == 0) {
                        follower.followPath(paths.shootTo1(), 0.9, false);

                        if (follower.atParametricEnd() || waitSecs(6)) {
                                spikeMark += 1;
                                resetActionTimer();
                                pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark == 1) {
                        follower.followPath(paths.shootTo2(), 0.9, false);

                        if (follower.atParametricEnd() || waitSecs(1)){
                            spikeMark += 1;
                            resetActionTimer();
                            pathState = PathState.OUT;
                        }
                    }
                }
                break;

            case DETECT:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }

                if (count == 0) {
                    double[] results = limelight.update();
                    x = results[1];
                    y = results[0];
                }

                intake.transferOff();

                if(!follower.isBusy()){
                    count += 1;
                    if (x == 0 && y == 0) {
                        detect = false;
                        if( spikeMark == 3) {
                            follower.followPath(paths.shootTo3(), 0.9, false);
                        } else if( spikeMark == 4) {
                            follower.followPath(paths.shootTo4(), 0.9, false);
                        }

                        if (follower.atParametricEnd() || waitSecs(3)) {
                            resetActionTimer();
                            count = 0;
                            pathState = PathState.OUT;
                        }
                    } else if (x != 0 || y != 0){
                        if (count2 == 0) {
                            double newY = paths.shootPose2.getY() - y - 7;
                            if (newY < 9) {
                                newY = 9;
                            }
                            ballCollect = new Pose(130, newY);
                            count2 += 1;
                        }

                        follower.followPath(paths.to(ballCollect), 0.9, true);

                        if (follower.atParametricEnd() || waitSecs(3)) {
                            resetActionTimer();
                            count = 0;
                            count2 = 0;
                            detect = true;
                            pathState = PathState.OUT;
                        }
                    }
                }
                break;

            case OUT:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }

                intake.transferOff();

                if (!follower.isBusy()) {
                    if (spikeMark == 2) {

                        follower.followPath(paths.outSet(), 0.9, false);

                        if(follower.atParametricEnd() || waitSecs(1)) {
                            resetActionTimer();
                            pathState = PathState.IN;
                        }
                    } else if (spikeMark == 3 || spikeMark == 4) {
                        if (detect) {
                            if (count2 == 0){
                                newOut = new Pose(ballCollect.getX() - 10, ballCollect.getY());
                                count2 += 1;
                            }
                            follower.followPath(paths.outFrom(ballCollect, newOut), 0.9, false);

                            if (follower.atParametricEnd()) {
                                resetActionTimer();
                                count2 = 0;
                                pathState = PathState.IN;
                            }
                        } else {
                            if(spikeMark == 3 ) {
                                follower.followPath(paths.outSet2(), 0.9, false);
                            } else if (spikeMark == 4){
                                follower.followPath(paths.outSet(), 0.9, false);
                            }

                            if (follower.atParametricEnd()) {
                                resetActionTimer();
                                pathState = PathState.IN;
                            }
                        }
                    }
                }
                break;

            case IN:
                intake.transferOff();

                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }

                if (!follower.isBusy()) {
                    if (spikeMark == 2) {
                        follower.followPath(paths.inSet(), 0.9, false);

                        if(follower.atParametricEnd() || waitSecs(1)) {
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else if (spikeMark == 3 || spikeMark == 4) {
                        if (detect) {
                            follower.followPath(paths.outFrom(newOut, ballCollect), 0.9, false);

                            if (follower.atParametricEnd()) {
                                resetActionTimer();
                                pathState = PathState.TO_SHOOT;
                            }
                        } else{
                            if(spikeMark == 3) {
                                follower.followPath(paths.inSet2(), 0.9, false);
                            } else if(spikeMark == 4){
                                follower.followPath(paths.inSet(), 0.9, false);
                            }

                            if (follower.atParametricEnd()) {
                                resetActionTimer();
                                pathState = PathState.TO_SHOOT;
                            }
                        }
                    }
                }
                break;

            case TO_SHOOT:
                if(!follower.isBusy()) {
                    if (spikeMark == 1) {
                        shooter.rotateTurretOld(68);
                        if (shootCount == 0) {
                            follower.followPath(paths.collectToShootNotSetFirst(), 0.8, false);
                            shootCount += 1;
                        }

                        if ( follower.atParametricEnd() ) {
                            resetActionTimer();
                            shootCount = 0;
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 2) {
                        shooter.rotateTurretOld(62);
                        if (shootCount == 0) {
                            follower.followPath(paths.collectToShootNotSet(), 0.8, false);
                            shootCount += 1;
                        }

                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            shootCount = 0;
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 3 || spikeMark == 4) {
                        shooter.rotateTurretOld(62);

                        if (shootCount == 0) {
                            follower.followPath(paths.collectToShootNotSet(), 0.8, false);
                            shootCount += 1;
                        }

                        if (follower.atParametricEnd()) {
                            resetActionTimer();
                            shootCount = 0;
                            stuckRun = 0;
                            pathState = PathState.SHOOT;
                        } else if (follower.isRobotStuck()){
                            if(stuckRun == 0) {
                                follower.followPath(paths.unstuck(), 0.8, false);//test this
                                stuckRun += 1;
                            }
                            if(follower.atParametricEnd()){
                                shootCount = 0;
                            }

                        }
                    }
                }
                break;

            case PARK:
                if(!follower.isBusy()) {
                    park();
                }
                break;

        }
    }

    private void park(){
        done = true;
        shooter.off();
        intake.intakeOff();
        intake.transferOff();
        shooter.rotateTurretOld(0);
    }

    @Override
    public void init() {
        // timer init
        actionTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        limelight = new BallDetection(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();

        shooter.setHood(hoodHeight);
        //shooter.rotateTurret(75);
    }

    public void init_loop(){
        choose.allianceInit(); // gets alliance
        alliance = choose.getSelectedAlliance(); // sets alliance
        telemetry.update();
    }

    public void start() { // on start
        // path setting
        follower = Constants.createFollower(hardwareMap);
        paths = new FarPaths(follower);
        follower.setStartingPose(paths.startPose);

        // type auto setting
        isMirror = paths.bluePath(alliance);

        // setting shooter stuff
        if (isMirror) { turnTableAngle = -75; }
        shooter.rotateTurretOld(65);

        // resets timers
        runtime.reset();
        resetActionTimer();

        // sets state
        pathState = PathState.START;
    }

    @Override
    public void loop() {
        if(!done) { shooter.far(); } // sets shooter speed

        follower.update(); // updates follower

        autonomousPathUpdate();//main auto code

        // auto control prints
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        telemetry.addData("inPark", inPark);

        // curr pos
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());

        // auto init prints
        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);

        // shooter prints
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());

        // updates and sends to phone
        telemetry.update();
        //dash.update();
    }


}