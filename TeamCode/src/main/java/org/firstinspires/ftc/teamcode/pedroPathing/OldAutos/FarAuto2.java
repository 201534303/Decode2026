package org.firstinspires.ftc.teamcode.pedroPathing.OldAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.BallDetection.BallDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

//@Autonomous(name = "FarAuto")
@Disabled
public class FarAuto2 extends OpMode {
    private Follower follower;
    private Timer actionTimer;
    private FarPaths paths;
    private Telemetry dash;


    private IntakeAuto intake;
    private ShooterAuto shooter;
    private OLDChoose choose;
    private ElapsedTime runtime = new ElapsedTime();

    private BallDetection limelight;
    private int spikeMark = 0;
    private int count = 0;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private boolean done = false;

    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK,
        OUT, IN, DETECT, WAIT, TEST
    }
    private boolean isMirror = false;
    private double turnTableAngle = 68;
    private double hoodHeight = 0.3;
    private boolean pathJustStarted = false;
    double x;
    double y;

    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                //shooter.rotateTurret(turnTableAngle);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case DETECT:
                intake.intakeIn();
                if(!follower.isBusy()){
                    //if(waitSecs(1)) {
                        if (count >= 1) {
                            follower.followPath(paths.shootTo12(), 0.8, true);
                            count = 0;

                            resetActionTimer();
                            pathState = PathState.WAIT;
                        } else {
                            if (x == 0 && y == 0) {
                                count += 1;
                                resetActionTimer();
                            } else {
                                //follower.followPath(paths.to(x, y), 0.8, true);
                                resetActionTimer();
                                pathState = PathState.WAIT;
                            }
                        }
                    //}
                }
                break;

            case WAIT:
                intake.intakeIn();

                if (!follower.isBusy() || waitSecs(6)) {
                    if (!intake.haveBall()) {
                        resetActionTimer();
                        pathState = PathState.OUT;
                    } else {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case TO_SHOOT:
                shooter.rotateTurret(turnTableAngle);

                if(!follower.isBusy()) {
                    if (spikeMark == 1) {
                        if (waitSecs(5)) {
                            follower.followPath(paths.collectToShoot(), 0.9, true);
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (spikeMark == 2) {
                        if (waitSecs(5)) {
                            follower.followPath(paths.collectToShoot2(), 0.9, true);
                            resetActionTimer();
                            pathState = PathState.SHOOT;
                        }
                    } else if (waitSecs(2)) {
                        follower.followPath(paths.collectToShoot2(), 0.9, true);
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    }
                }
                break;

            case SHOOT:
                if ( !follower.isBusy() && waitSecs(1)) {
                    intake.allTheWaySlow();//go all the way to shoot
                    resetActionTimer();
                    pathState = PathState.INTAKE;
                }
                break;

            case INTAKE:
                if (!follower.isBusy() && waitSecs(1.25)) {//waits 0.5 works
                    spikeMark += 1;
                    intake.intakeIn();
                    intake.transferOff();

                    if (spikeMark == 1) {
                        follower.followPath(paths.shootTo1(), 0.8, false);
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (spikeMark == 2) {
                        follower.followPath(paths.shootTo2(), 0.8, false);
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (spikeMark < 5 && spikeMark > 2){
                        count = 0;
                        resetActionTimer();
                        pathState = PathState.DETECT;
                    } else if (spikeMark >= 6){
                        done = true;
                        follower.followPath(paths.shootToPark(), 0.75, false);
                        pathState = PathState.PARK;
                    }
                }
                break;

            case OUT:
                if (!follower.isBusy()) {
                    follower.followPath(paths.outSet(), 0.8, false);
                    if (!intake.haveBall()) {
                        resetActionTimer();
                        pathState = PathState.IN;
                    } else {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case IN:
                if (!follower.isBusy()) {
                    follower.followPath(paths.inSet(), 0.8, false);
                    if (!intake.haveBall() && waitSecs(1)) {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (intake.haveBall()) {
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case PARK:
                if(!follower.isBusy()) {
                    shooter.off();
                    intake.intakeOff();
                    intake.transferOff();
                    shooter.rotateTurret(0);
                }
                break;
        }
    }

    @Override
    public void init() {
        actionTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        limelight = new BallDetection(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();

        shooter.setHood(hoodHeight);
        //shooter.rotateTurret(75);
        resetActionTimer();
    }

    public void init_loop(){
        choose.allianceInit();
        alliance = choose.getSelectedAlliance();
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!done) { shooter.far(); }

        follower.update();

        autonomousPathUpdate();//main auto code

        double[] results = limelight.update();
        x = results[1];
        y = results[0];
        telemetry.addData("detect x", x);
        telemetry.addData("detect y", y);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("x add", follower.getPose().getX() + x);
        telemetry.addData("y add", follower.getPose().getY() - y);

        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new FarPaths(follower);

        isMirror = paths.bluePath(alliance);
        follower.setStartingPose(paths.startPose);

        if (isMirror) { turnTableAngle = -75; }
        shooter.rotateTurret(67);

        runtime.reset();
        pathState = PathState.START;
    }
}
