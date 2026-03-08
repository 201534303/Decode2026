package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.FarAutoAttempt2.PathState.IN;
import static org.firstinspires.ftc.teamcode.auto.FarAutoAttempt2.PathState.TO_SHOOT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.LimelightV5;
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OldAutos.FarAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "FarAuto")

public class FarAutoAttempt2 extends OpMode {
    // Robot Subsystems
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private LimelightV5 limelight;
    private RobotActions robotActions;


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
        DETECT, ONE_MORE_TIME, TWO_MORE_TIME, TEST,
        OUT, IN
    }
    PathState pathState = PathState.START;

    // Random Variables
    private int spikeMark = 0;
    private boolean done = false;
    Pose ballCollect;
    private int shootCount = 0;
    private double turnTableAngle = 74;
    double newY;
    int posCount = 0;
    int negCount = 0;
    double average = 0;
    double posAverage = 0;
    double negAverage = 0;
    double offset = 40;

    private double timeDif = 1.0;
    private ElapsedTime overallRuntime;
    private double lastTime;
    private boolean outPathSet = false;
    private boolean inPathSet = false;

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

            case TEST:
                break;

            case SHOOT:
                if (!follower.isBusy()) {
                    if(waitSecs(0.5)){
                        intake.allTheWaySlow();// go all the way to shoot

                        if (spikeMark == 0) {
                            if (waitSecs(1.35)) {//1.75
                                resetActionTimer();
                                intakePathSet = false;
                                pathState = PathState.INTAKE;
                            }
                        } else if (spikeMark == 1 || spikeMark == 5) {
                            if (waitSecs(1.35)) {//2.5
                                resetActionTimer();
                                intakePathSet = false;
                                pathState = PathState.INTAKE;
                            }
                        } else if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4) {
                            if (waitSecs(1.5)) { // waits 1 sec to wait for all balls to shoot
                                resetActionTimer();
                                spikeMark += 1;
                                pathState = PathState.DETECT;
                                detectInitDone = false;
                                detectPathSet = false;
                            }
                        }
                    }
                }
                break;

            case INTAKE:
                if (intake.haveBall() && waitSecs(0.5)){
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
                            follower.followPath(paths.shootTo1(), 0.9, true);
                        } else if (spikeMark == 1) {
                            follower.followPath(paths.shootTo2(), 0.9, true);
                        } else if (spikeMark == 5) {
                            park();
                            follower.followPath(paths.shootToPark(), 0.6, true);
                            pathState = PathState.PARK;
                            break;
                        }
                    }

                    if (spikeMark == 1 && (follower.atParametricEnd() && waitSecs(1) || waitSecs(1.5))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.OUT;
                    } else if (spikeMark == 0 && (follower.atParametricEnd() && waitSecs(2.5) || waitSecs(3))) {
                        spikeMark += 1;
                        intakePathSet = false;
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case DETECT:
                intake.intakeIn();
                intake.transferOff();

                if (intake.haveBall() && waitSecs(0.5)){
                    resetActionTimer();
                    detectInitDone = false;
                    detectPathSet = false;
                    average = 0;
                    posCount = 0;
                    posAverage = 0;
                    negCount = 0;
                    negAverage = 0;
                    pathState = PathState.TO_SHOOT;
                }

                if (!detectInitDone) {
                    ArrayList<double[]> detections = limelight.updateBall2(timeDif);

                    if (detections != null && !detections.isEmpty()) {
                        ArrayList<Double> results = new ArrayList<>();

                        for (double[] row : detections) {
                            double camX = row[0];
                            results.add(camX);
                        }

                        for (double distance : results) {
                            if (distance > 0) {
                                posCount++;
                                posAverage += distance;
                            } else {
                                negCount++;
                                negAverage += distance;
                            }
                        }

                        if (negCount > posCount) {
                            average = (negCount > 0) ? negAverage / negCount : 0;
                        } else {
                            average = (posCount > 0) ? posAverage / posCount : 0;
                        }
                    }
                    detectInitDone = true;
                }

                if(!follower.isBusy() && !detectPathSet && detectInitDone){
                    detectPathSet = true;

                    if (posCount == 0 && negCount == 0) {
                        if(spikeMark == 3 || spikeMark == 5){
                            follower.followPath(paths.shootTo3(), 0.9, true);
                        } else {
                            follower.followPath(paths.shootTo4(), 0.9, true);
                        }
                    } else {
                        if (alliance == OLDChoose.Alliance.BLUE){
                            average = -average;
                        }
                        newY = paths.shootPose2.getY() + average;
                        if (newY < 9) { newY = 9; }
                        else if (newY > 35) { newY = 35; }

                        ballCollect = new Pose(130, newY, 0); // ADD THIS BACK
                        if(alliance == OLDChoose.Alliance.BLUE){
                            ballCollect = ballCollect.mirror();
                        }

                        double checkY = ballCollect.getY(); // use mirrored Y for the check
                        if (checkY < 12){ follower.followPath(paths.shootTo4(), 0.9, true); }
                        else { follower.followPath(paths. to(ballCollect), 0.9, true); }
                    }
                }

                if (follower.atParametricEnd() && waitSecs(2) || waitSecs(3)) {//1.5/
                    resetActionTimer();
                    detectInitDone = false;
                    detectPathSet = false;
                    average = 0;
                    posCount = 0;
                    posAverage = 0;
                    negCount = 0;
                    negAverage = 0;
                    pathState = PathState.TO_SHOOT;
                }
                break;

            case TO_SHOOT:
                if (shootCount == 0) {
                    intake.intakeIn();
                    follower.followPath(paths.collectToShootNotSet(), 0.9, true);
                    shootCount += 1;
                }

                if (spikeMark == 2) {
                    if (waitSecs(0.5)) {
                        intake.setIntakeSpeed(0.3);
                    }
                } else if (spikeMark == 3 || spikeMark == 4 || spikeMark == 5) {
                    if (waitSecs(0.4)) {
                        intake.setIntakeSpeed(0);
                    }
                } else {
                    if(waitSecs(0.5)) {
                        intake.setIntakeSpeed(0);
                    }
                }

                if (follower.atParametricEnd()) {
                    resetActionTimer();
                    shootCount = 0;
                    pathState = PathState.SHOOT;
                }
                break;

            case OUT:
                intake.setIntakeSpeed(-0.3);

                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }
                if (!outPathSet) {
                    outPathSet = true;
                    follower.followPath(paths.outSet(), 0.75, true);
                }

                if( (follower.atParametricEnd() && waitSecs(0.5)) || waitSecs(0.75) ){
                    resetActionTimer();
                    outPathSet = false;
                    pathState = IN;
                }
                break;

            case IN:
                if (intake.haveBall()){
                    resetActionTimer();
                    pathState = PathState.TO_SHOOT;
                }
                intake.intakeIn();
                if (!inPathSet) {
                    inPathSet = true;
                    follower.followPath(paths.inSet(), 0.75, true);
                }

                if((follower.atParametricEnd() && waitSecs(1)) || waitSecs(1.25)){
                    resetActionTimer();
                    inPathSet = false;
                    pathState = TO_SHOOT;
                }
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
        limelight = new LimelightV5(hardwareMap, 2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
        overallRuntime = new ElapsedTime();

        shooter.setHood(0.2);
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
        robotActions = new RobotActions(shooter, follower, telemetry);

        // type auto setting
        isMirror = paths.bluePath(alliance);
        follower.setStartingPose(paths.startPose);

        // setting shooter stuff
        if (isMirror) {
            offset = 0;
            turnTableAngle = -71;
        }
        shooter.rotateTurret(turnTableAngle);

        // resets timers
        runtime.reset();
        overallRuntime.reset();
        resetActionTimer();

        // sets state
        pathState = PathState.START;
    }

    @Override
    public void loop() {
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = nowTime - lastTime;
        lastTime = nowTime;
        telemetry.addData("loop time", timeDif);

        follower.update(); // updates follower

        if(spikeMark == 0 && !done) { shooter.farFaster(); } // sets shooter speed
        if(!done && spikeMark != 0) {
            //shooter.far();
            robotActions.updateTurret(alliance, (follower.getPose().getX() + offset), follower.getPose().getY(), follower.getHeading());
            robotActions.updateShooter(alliance, (follower.getPose().getX()), follower.getPose().getY(), 0);
        } // sets shooter speed

        autonomousPathUpdate();//main auto code

        telemetry.addData("average", average);
        telemetry.addData("posCount", posCount);
        telemetry.addData("negCoung", negCount);
        telemetry.addData("posAverage", posAverage);
        telemetry.addData("negAverage", negAverage);

        //telemetry.addData("turntable", shooter.thetaT);
        //telemetry.addData("headingError", Math.toDegrees(follower.getHeadingError()));

        // auto init prints
        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);

        // auto control prints
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        //telemetry.addData("inPark", inPark);

        // curr pos
        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());

        // shooter prints
        //telemetry.addData("heading", follower.getPose().getHeading());
        //telemetry.addData("flywheel RPM", shooter.getMotorRPM());

        // updates and sends to phone
        telemetry.update();
        dash.update();
    }

    @Override
    public void stop() {
        for (int i  = 0; i < 50; i++){
            Pose p = follower.getPose();
            PoseSaver.save(p.getX(), p.getY(), p.getHeading());
            follower.update();
        }
    }

}