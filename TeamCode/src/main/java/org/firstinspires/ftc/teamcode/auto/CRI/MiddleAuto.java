package org.firstinspires.ftc.teamcode.auto.CRI;

import static org.firstinspires.ftc.teamcode.auto.CompSeason.FarAutoAttempt2.PathState.DETECT;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI.CRI_Far_Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.CRI.MiddlePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "MiddleAuto")

public class MiddleAuto extends OpMode {
    private static final double AUTO_BLUE_LIGHT = 0.63;
    private static final double AUTO_RED_LIGHT = 0.28;
    private double timeVisionHelper;


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
    private MiddlePaths paths;
    private Telemetry dash;
    private boolean detectInitDone = false;
    private boolean detectPathSet = false;
    private boolean intakePathSet = false;
    private boolean firstShootPathSet = false;
    private boolean shootMove;

    // Actions
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK,
        DETECT, INTAKE_PT2
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

    private void transitionTo(PathState newState) {
        resetActionTimer();
        pathState = newState;
    }
    private boolean waitForPathEndOrTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() || waitSecs(timeoutSecs));
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
                intake.allTheWay();// go all the way to shoot

                if(spikeMark == 0){
                    if(waitSecs(1.5)) {
                        intakePathSet = false;
                        transitionTo(PathState.INTAKE_PT2);
                    }
                }
                break;

            case INTAKE_PT2:
                if (!intakePathSet) {
                    intakePathSet = true;
                    follower.followPath(paths.notHit(), 1, false);
                }
                if(waitForPathEndOrTimeout(2)){
                    resetDetectionState();
                    transitionTo(PathState.PARK);
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

        shooter.setHood(.55);
    }

    public void init_loop(){
        choose.allianceInit(); // gets alliance
        alliance = choose.getSelectedAlliance(); // sets alliance

        isMirror = (alliance == OLDChoose.Alliance.BLUE);
        indicatorLight.setPosition(isMirror ? AUTO_BLUE_LIGHT : AUTO_RED_LIGHT);

        turnTableAngle = isMirror ? -45 : 50;

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
        paths = new MiddlePaths(follower);
        robotActions = new RobotActions(shooter, follower, telemetry);

        // type auto setting
        isMirror = paths.redPath(alliance);
        follower.setStartingPose(paths.startPose);

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
                shooter.set(1200);
            }
        }

        autonomousPathUpdate();//main auto code

        telemetry.addData("average", average);
        telemetry.addData("posCount", posCount);
        telemetry.addData("negCoung", negCount);
        telemetry.addData("posAverage", posAverage);
        telemetry.addData("negAverage", negAverage);

        // auto init prints
        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);

        // auto control prints
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);

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
}
