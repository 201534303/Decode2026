package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.FarAutoAttempt2.PathState.IN;
import static org.firstinspires.ftc.teamcode.auto.FarAutoAttempt2.PathState.TO_SHOOT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.LimelightV5;
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "FarAuto")

public class FarAutoAttempt2 extends OpMode {
    private static final double AUTO_BLUE_LIGHT = 0.63;
    private static final double AUTO_RED_LIGHT = 0.28;

    // Robot Subsystems
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private LimelightV5 limelight;
    private RobotActions robotActions;
    private Servo indicatorLight;


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
    int posCount = 0;
    int negCount = 0;
    double average = 0;
    double posAverage = 0;
    double negAverage = 0;
    //double offset = 40;

    private double timeDif = 1.0;
    private ElapsedTime overallRuntime;
    private double lastTime;
    private boolean outPathSet = false;
    private boolean inPathSet = false;
    private double x, y, heading;

    // Timer Control
    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    private void resetDetectionState() {
        detectInitDone = false;
        detectPathSet = false;
        average = 0;
        posCount = 0;
        posAverage = 0;
        negCount = 0;
        negAverage = 0;
    }

    private void transitionTo(PathState newState) {
        resetActionTimer();
        pathState = newState;
    }

    private boolean waitForPathEndOrTimeout(double endDelaySecs, double timeoutSecs) {
        return (follower.atParametricEnd() && waitSecs(endDelaySecs)) || waitSecs(timeoutSecs);
    }

    private void stopIntakeAndTransfer() {
        intake.transferOff();
        intake.intakeIn();
    }

    private double detectionAverageOffset() {
        return alliance == OLDChoose.Alliance.BLUE ? -average : average;
    }

    private void followDetectionPath() {
        if (posCount == 0 && negCount == 0) {
            if(spikeMark == 3) {
                ballCollect = paths.ballCollect12;
            } else {
                ballCollect = paths.ballCollect2;
            }
            follower.followPath(spikeMark == 3 ? paths.shootTo3() : paths.shootTo4(), 1, true);
            return;
        }

        ballCollect = paths.detectionCollectPose(detectionAverageOffset(), alliance);
        if (paths.shouldUseDetectionFallback(ballCollect)) {
            ballCollect = paths.ballCollect2;
            follower.followPath(paths.shootTo4(), 1, true);
        } else {
            follower.followPath(paths.to(ballCollect), 1, true);
        }
    }

    private void updateDetectionAverage(ArrayList<double[]> detections) {
        ArrayList<Double> results = new ArrayList<>();
        for (double[] row : detections) {
            results.add(row[0]);
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

    // Main Auto Code
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                if (waitSecs(1.5)) { //1.25
                    transitionTo(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if (!follower.isBusy() /*&& waitSecs(0.5)*/) {
                    intake.allTheWay();// go all the way to shoot

                    if (spikeMark == 0) {
                        if (waitSecs(1.25)) {//1
                            intakePathSet = false;
                            transitionTo(PathState.INTAKE);
                        }
                    } else if (spikeMark == 1 || spikeMark == 5) {
                        if (waitSecs(1)) {//1
                            intakePathSet = false;
                            transitionTo(PathState.INTAKE);
                        }
                    } else if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4 ) {
                        if (waitSecs(1)) { // 1
                            spikeMark += 1;
                            resetDetectionState();
                            transitionTo(PathState.DETECT);
                        }
                    }
                }
                break;

            case INTAKE:
                intake.transferOff();
                if (intake.haveBall() && waitSecs(0.5)){
                    spikeMark ++;
                    transitionTo(PathState.TO_SHOOT);
                    break;
                } else if (!follower.isBusy()) {
                    if (!intakePathSet) {
                        intakePathSet = true;
                        if (spikeMark == 0) {
                            follower.followPath(paths.shootTo1(), 1, true);
                        } else if (spikeMark == 1) {
                            follower.followPath(paths.shootTo2(), 1, true);
                        } else if (spikeMark == 5) {
                            park();
                            follower.followPath(paths.shootToPark(), 0.6, true);
                            pathState = PathState.PARK;
                            break;
                        }
                    }

                    if (spikeMark == 1 && waitForPathEndOrTimeout(0.54, 0.65)) {
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.OUT);
                    } else if (spikeMark == 0 && waitForPathEndOrTimeout(2.15, 2.4)) {
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    }
                }
                break;

            case DETECT:
                intake.intakeIn();
                intake.transferOff();

                if (intake.haveBall() && waitSecs(0.5)){
                    resetDetectionState();
                    transitionTo(PathState.TO_SHOOT);
                }

                if (!detectInitDone) {
                    ArrayList<double[]> detections = limelight.updateBall2(timeDif);

                    if (detections != null && !detections.isEmpty()) {
                        updateDetectionAverage(detections);
                    }
                    detectInitDone = true;
                }

                if(!follower.isBusy() && !detectPathSet && detectInitDone){
                    detectPathSet = true;
                    followDetectionPath();
                }

                if (waitForPathEndOrTimeout(1.75, 2)) {//2, 2.25
                    resetDetectionState();
                    //transitionTo(PathState.OUT);
                    transitionTo(PathState.TO_SHOOT);
                }
                break;

            case TO_SHOOT:

                if (shootCount == 0) {
                    intake.intakeIn();
                    follower.followPath(paths.collectToShootNotSet(), 1, true);
                    shootCount += 1;
                }

                if (spikeMark == 2) {
                    if (waitSecs(0.5)) {
                       intake.setIntakeSpeed(0.3);
                    }
                } else if (spikeMark == 3 || spikeMark == 4) {
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
                if (intake.haveBall()){
                    transitionTo(PathState.TO_SHOOT);
                }
                if (!outPathSet) {
                    outPathSet = true;
                    if(spikeMark == 2) {
                        follower.followPath(paths.outSet(), 0.75, true);
                    } else{
                        follower.followPath(paths.outNotSet(ballCollect, alliance), 0.75, true);
                    }
                }

                if(waitForPathEndOrTimeout(0.25, 0.5)){
                    outPathSet = false;
                    transitionTo(IN);
                }
                break;

            case IN:
                if (intake.haveBall()){
                    transitionTo(PathState.TO_SHOOT);
                }
                intake.intakeIn();
                if (!inPathSet) {
                    inPathSet = true;
                    if(spikeMark == 2) {
                        follower.followPath(paths.inSet(), 0.75, true);
                    } else{
                        follower.followPath(paths.inNotSet(ballCollect), 0.75, true);

                    }
                }

                if(waitForPathEndOrTimeout(0.75, 1)){
                    inPathSet = false;
                    transitionTo(TO_SHOOT);
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
        indicatorLight = hardwareMap.get(Servo.class, "taillight");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
        overallRuntime = new ElapsedTime();

        shooter.setHood(0);
    }

    public void init_loop(){
        choose.allianceInit(); // gets alliance
        alliance = choose.getSelectedAlliance(); // sets alliance

        isMirror = (alliance == OLDChoose.Alliance.BLUE);
        indicatorLight.setPosition(isMirror ? AUTO_BLUE_LIGHT : AUTO_RED_LIGHT);

        turnTableAngle = isMirror ? -71 : 72;
        shooter.rotateTurret(turnTableAngle);

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

        Pose position = follower.getPose();
        x = position.getX();
        y = position.getY();
        heading = position.getHeading();

        if(!done) {
            if (spikeMark == 0) {
                shooter.farFaster();
            } else {
                shooter.far();
//                if(alliance == OLDChoose.Alliance.RED){
//                    x += 100;
//                }
                robotActions.updateTurret(alliance, x, y, heading);
            } // sets shooter speed
        }

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
