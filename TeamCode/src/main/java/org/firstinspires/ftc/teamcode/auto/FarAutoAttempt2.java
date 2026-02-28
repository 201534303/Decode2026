package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.LimelightV5;
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
    private LimelightV5 limelight;

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
    private Boolean timerOnce = false;
    private boolean detectInitDone = false;
    private boolean detectPathSet = false;
    private boolean intakePathSet = false;

    // Actions
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK,
        OUT, IN, DETECT, ONE_MORE_TIME,
        SIDE_SHUFFLE, FIX
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
    private double turnTableAngle = 74;
    private double turnTableAngle2 = 75;
    private double turnTableAngle3 = 65;

    double newY;

    // For Vision
    double x;
    double y;
    int count3 = 0;

    double xLast;
    double yLast;
    Pose currPose;

    // Timer Control
    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    // Main Auto Code
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                shooter.far();
                if (waitSecs(1)) { //1.25
                    resetActionTimer(); // resets timer
                    pathState = PathState.SHOOT; // sets to shoot state
                }
                break;

            case SHOOT:
                if (!follower.isBusy()) {
                    if(waitSecs(0.75)){
                        intake.allTheWaySlow();// go all the way to shoot

                        if (spikeMark == 0) {
                            if (waitSecs(1.5)) {//1.75
                                resetActionTimer();
                                intakePathSet = false;
                                pathState = PathState.INTAKE;
                            }
                        } else if (spikeMark == 1 || spikeMark == 5) {
                            if (waitSecs(1.75)) {//2.5
                                resetActionTimer();
                                intakePathSet = false;
                                pathState = PathState.INTAKE;
                            }
                        } else if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4) {
                            if (waitSecs(1.75)) { // waits 1 sec to wait for all balls to shoot
                                resetActionTimer();
                                pathState = PathState.DETECT;
                                detectInitDone = false;
                                detectPathSet = false;
                            }
                        }
                    }
                }
                break;

            case INTAKE:
                if (intake.haveBall() && waitSecs(1)){
                    timerOnce = true;
                    resetActionTimer();
                    spikeMark += 1;
                    pathState = PathState.TO_SHOOT;
                } else if (!follower.isBusy()) {
                    intake.transferOff();
                    intake.intakeIn();

                    if (!intakePathSet) {
                        intakePathSet = true;
                        if (spikeMark == 0) {
                            follower.followPath(paths.shootTo1(), 0.9, false);
                        } else if (spikeMark == 1) {
                            follower.followPath(paths.shootTo2(), 0.9, false);
                        } else if (spikeMark == 5) {
                            park();
                            follower.followPath(paths.shootToPark(), 1, false);
                            pathState = PathState.PARK;
                            break;
                        }
                    }

                    if (spikeMark == 0 && (follower.atParametricEnd() || waitSecs(6))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (spikeMark == 1 && (follower.atParametricEnd() && waitSecs(2) || waitSecs(3))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case DETECT:
                if (intake.haveBall() && waitSecs(1)){
                    resetActionTimer();
                    detectInitDone = false;
                    detectPathSet = false;
                    pathState = PathState.TO_SHOOT;
                    break;
                }

                if (!detectInitDone) {
                    spikeMark += 1;
                    double[] results = limelight.updateBall();
                    xLast = results[1];
                    yLast = results[0];
                    detectInitDone = true;
                }

                intake.transferOff();

                if(!follower.isBusy() && !detectPathSet){
                    detectPathSet = true;

                    if (xLast == 0 && yLast == 0 || alliance == OLDChoose.Alliance.BLUE) {
                        detect = false;
                        if(spikeMark == 3 || spikeMark == 5){
                            follower.followPath(paths.shootTo3(), 0.9, false);
                        } else {
                            follower.followPath(paths.shootTo4(), 0.9, false);
                        }
                    } else /*if (xLast != 0 || yLast != 0)*/{
                        newY = paths.shootPose2.getY() - yLast - 7;
                        if (newY < 9) { newY = 9; }

                        ballCollect = new Pose(130, newY, 0); // ADD THIS BACK

                        if (newY < 12){ follower.followPath(paths.shootTo4(), 0.9, false); }
                        else { follower.followPath(paths. to(ballCollect), 0.9, true); }
                    }
                }

                if (follower.atParametricEnd() && waitSecs(2) || waitSecs(3)) {//1.5/
                    resetActionTimer();
                    detectInitDone = false;
                    detectPathSet = false;
                    detect = true;
                    pathState = PathState.TO_SHOOT;
                }
                break;

            /*case ONE_MORE_TIME:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }

                if (count == 0) {
                    double[] results = limelight.updateBall();
                    x = results[1];
                    y = results[0];
                    count += 1;
                }

                //intake.transferOff();

                //if(!follower.isBusy()) {
                    if (xLast != 0 || yLast != 0) {
                        if (count2 == 0) {
                            double diffy = Math.abs(yLast - y);
                            newY = follower.getPose().getY() - y - diffy; // 7

                            if (newY < 9) {
                                newY = 9;
                            }

                            ballCollect = new Pose(130, newY, 0);
                            count2 += 1;
                            currPose = follower.getPose();
                        }

                        if(count3 == 0){
                            if (newY < 12) { // 20
                                follower.followPath(paths.shootTo4(currPose), 0.9, false);
                            } else {
                                follower.followPath(paths.to(currPose, ballCollect), 0.9, true);
                            }
                            count3 += 1;
                        }


                        if (follower.atParametricEnd() && waitSecs(1)|| waitSecs(3)) {
                            count = 0;
                            count2 = 0;
                            detect = true;
                            count3 = 0;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    } else{
                        if(follower.atParametricEnd() && waitSecs(1) || waitSecs(3)){
                            count3 = 0;
                            count = 0;
                            count2 = 0;
                            detect = true;
                            resetActionTimer();
                            pathState = PathState.TO_SHOOT;
                        }
                    }
                //}
                break;*/

            /*case OUT:
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
                                if(alliance == OLDChoose.Alliance.RED) {
                                    newOut = new Pose(ballCollect.getX() - 10, ballCollect.getY(), ballCollect.getHeading());
                                } /*else if(alliance == OLDChoose.Alliance.BLUE) {
                                    newOut = new Pose(ballCollect.getX() + 10, ballCollect.getY(), ballCollect.getHeading());
                                }
                                count2 += 1;
                            }
                            follower.followPath(paths.outFrom(ballCollect, newOut), 0.9, false);

                            if (follower.atParametricEnd() || waitSecs(1)) {
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

                            if (follower.atParametricEnd() || waitSecs(1)) {
                                resetActionTimer();
                                pathState = PathState.IN;
                            }
                        }
                    }
                }
                break; */

            /*case IN:
                intake.transferOff();
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

                            if (follower.atParametricEnd() || waitSecs(1)) {
                                resetActionTimer();
                                pathState = PathState.TO_SHOOT;
                            }
                        } else{
                            if(spikeMark == 3) {
                                follower.followPath(paths.inSet2(), 0.9, false);
                            } else if(spikeMark == 4){
                                follower.followPath(paths.inSet(), 0.9, false);
                            }

                            if (follower.atParametricEnd() || waitSecs(1)) {
                                resetActionTimer();
                                pathState = PathState.TO_SHOOT;
                            }
                        }
                    }
                }
                break;*/

            case TO_SHOOT:
                //if(!follower.isBusy()) {
                    if (shootCount == 0) {
                        follower.followPath(paths.collectToShootNotSet(), 0.9, true);
                        shootCount += 1;
                    }

                    if (spikeMark == 1) {
                        shooter.rotateTurret(75);
                    } else if (spikeMark == 2) {
                        shooter.rotateTurret(75);
                        if (waitSecs(0.5)) {
                            intake.setIntakeSpeed(0.3);
                        }
                    } else if (spikeMark == 3) {
                        shooter.rotateTurret(74);
                        if (waitSecs(0.75)) {
                            intake.setIntakeSpeed(0.3);
                        }
                    } else if (spikeMark == 4) {
                        shooter.rotateTurret(74);
                        if (waitSecs(0.75)) {
                            intake.setIntakeSpeed(0.3);
                        }
                    } else if (spikeMark == 5) {
                        shooter.rotateTurret(74);
                        if (waitSecs(0.75)) {
                            intake.setIntakeSpeed(0.3);
                        }
                    } else {
                        shooter.rotateTurret(75);

                        if(waitSecs(0.5)) {
                            intake.setIntakeSpeed(0);
                        }
                    }

                if (follower.atParametricEnd()) {
                    resetActionTimer();
                    shootCount = 0;
                    pathState = PathState.SHOOT;
                }
                //}
                break;

            case PARK:
                if (!follower.isBusy()) {
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
        shooter.rotateTurret(0);
    }

    @Override
    public void init() {
        // timer init
        actionTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        limelight = new LimelightV5(hardwareMap, 0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();

        shooter.setHood(0.2);
        shooter.rotateTurret(100);

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

        // type auto setting
        isMirror = paths.bluePath(alliance);
        follower.setStartingPose(paths.startPose);

        // setting shooter stuff
        if (isMirror) {
            turnTableAngle = -65;
            turnTableAngle2 = -68;
        }
        shooter.rotateTurret(turnTableAngle);

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

        telemetry.addData("turntable", shooter.thetaT);
        telemetry.addData("headingError", Math.toDegrees(follower.getHeadingError()));

        // auto init prints
        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);

        // auto control prints
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        telemetry.addData("inPark", inPark);

        // curr pos
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());

        // shooter prints
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());

        // updates and sends to phone
        telemetry.update();
        //dash.update();
    }


}