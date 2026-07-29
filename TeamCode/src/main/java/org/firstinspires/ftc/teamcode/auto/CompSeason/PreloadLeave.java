package org.firstinspires.ftc.teamcode.auto.CompSeason;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.LimelightV5;
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Disabled
public class PreloadLeave extends OpMode {
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
        DETECT, OUT, IN
    }
    PathState pathState = PathState.START;

    // Random Variables
    private int spikeMark = 0;
    private boolean done = false;
    Pose ballCollect;
    private boolean reset;
    private int shootCount = 0;
    private double turnTableAngle = 74;
    int posCount = 0;
    int negCount = 0;
    double average = 0;
    double posAverage = 0;
    double negAverage = 0;
    double offset = 10;

    private double timeDif = 1.0;
    private ElapsedTime overallRuntime;
    private double lastTime;
    private boolean outPathSet = false;
    private boolean inPathSet = false;
    private double x, y, heading;
    private double timeVisionHelper;

    // Timer Control
    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){
 return actionTimer.getElapsedTimeSeconds() > seconds; }

    private void resetDetectionState() {
        detectInitDone = false;
        detectPathSet = false;
        average = 0;
        posCount = 0;
        posAverage = 0;
        negCount = 0;
        negAverage = 0;
    }

    private double dist(double[] a, double[] b) {
        return Math.sqrt(Math.pow((a[0]-b[0]),2) + Math.pow((a[1]-b[1]),2));
    }
    private double tripletScore(double[] a, double[] b, double[] c) {
        return Math.max(dist(a, b), Math.max(dist(b, c), dist(a, c)));
    }
    public double[][] findBestScore(ArrayList<double[]> points) {
        double bestScore = Double.MAX_VALUE;
        double[][] bestTriplet = null;
        int n = points.size();

        for (int i = 0; i < n - 2; i++) {
            for (int j = i + 1; j < n - 1; j++) {
                for (int k = j + 1; k < n; k++) {
                    double[] p1 = points.get(i);
                    double[] p2 = points.get(j);
                    double[] p3 = points.get(k);
                    double score = tripletScore(p1, p2, p3);
                    if (score < bestScore) {
                        bestScore = score;
                        bestTriplet = new double[][] {p1, p2, p3};
                    }
                }
            }
        }
        return bestTriplet;
    }
    private void transitionTo(PathState newState) {
        resetActionTimer();
        pathState = newState;
    }

    private boolean waitForPathEndOrTimeout(double endDelaySecs, double timeoutSecs) {
        return (follower.atParametricEnd() && waitSecs(endDelaySecs)) || waitSecs(timeoutSecs);
    }

    private boolean waitForPathEndOrTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() || waitSecs(timeoutSecs));
    }

    private boolean waitForPathEndAndTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() && waitSecs(timeoutSecs));
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

    private void followDetectionMid() {
        if (posCount == 0 && negCount == 0) {
            return;
        }

        ballCollect = paths.detectionCollectPoseNotSet(detectionAverageOffset(), alliance);

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
        double[][] optimumPts = findBestScore(detections);

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
                if (waitSecs(1.35)) { //1.25
                    transitionTo(PathState.SHOOT);
                }
                break;

            case SHOOT:
                //if (!follower.isBusy() /*&& waitSecs(0.5)*/) {
                    intake.allTheWay();// go all the way to shoot

                    if (spikeMark == 0) {
                        if (waitSecs(0.5)) {//0.6
                            intakePathSet = false;
                            transitionTo(PathState.INTAKE);
                        }
                    }
                break;

            case INTAKE:
                intake.transferOff();

                //else if (!follower.isBusy()) {
                    if (!intakePathSet) {
                        intakePathSet = true;

                        park();
                        follower.followPath(paths.shootToPark(), 0.6, true);
                        pathState = PathState.PARK;
                        break;
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
        limelight = new LimelightV5(hardwareMap, 2);
        indicatorLight = hardwareMap.get(Servo.class, "taillight");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
        overallRuntime = new ElapsedTime();

        shooter.setHood(.55);
    }

    public void init_loop(){
        choose.allianceInit(); // gets alliance
        alliance = choose.getSelectedAlliance(); // sets alliance

        isMirror = (alliance == OLDChoose.Alliance.BLUE);
        indicatorLight.setPosition(isMirror ? AUTO_BLUE_LIGHT : AUTO_RED_LIGHT);

        turnTableAngle = isMirror ? -71 : 72;

        if(alliance == OLDChoose.Alliance.BLUE){
            offset = 0;
        } else if (alliance == OLDChoose.Alliance.RED){
            offset = 10;
        }
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
                robotActions.updateTurret(alliance, (x + offset), y, heading);
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
    int times = 0;
    @Override
    public void stop() {
        for (int i  = 0; i < 50; i++){
            Pose p = follower.getPose();
            PoseSaver.save(p.getX(), p.getY(), p.getHeading());
            follower.update();
        }
    }

    public boolean throttleVision(double curentTime){
        if (curentTime - timeVisionHelper > 100){
            return true;
        }
        return false;
    }


}
