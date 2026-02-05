package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "FarAuto")

public class FarAuto extends OpMode {
    private Follower follower;
    private Timer actionTimer;
    private FarPaths paths;

    private IntakeAuto intake;
    private ShooterAuto shooter;
    private OLDChoose choose;
    private ElapsedTime runtime = new ElapsedTime();

    private int spikeMark = 0;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private boolean done = false;

    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK,
        OUT, IN
    }
    private boolean isMirror = false;
    private double turnTableAngle = 63;
    private double hoodHeight = 0.2;

    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                shooter.rotateTurret(turnTableAngle);
                resetActionTimer();
                pathState = PathState.SHOOT;
                break;

            case TO_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(paths.collectToShoot(), 0.8, true);
                    resetActionTimer();
                    pathState = PathState.SHOOT;
                }
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    shooter.rotateTurret(turnTableAngle);
                    if (waitSecs(2)){
                        intake.allTheWaySlow();//go all the way to shoot
                        resetActionTimer();
                        pathState = PathState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                if (!follower.isBusy() && waitSecs(1.25)) {//waits 0.5 works
                    spikeMark += 1;
                    intake.intakeIn();
                    intake.transferOff();

                    if (spikeMark == 1) {
                        follower.followPath(paths.shootTo1(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.TO_SHOOT;
                    } else if (spikeMark < 5){
                        follower.followPath(paths.shootTo2(), 0.75, false);
                        resetActionTimer();
                        if (!intake.haveBall()) {
                            pathState = PathState.OUT;
                        } else {
                            pathState = PathState.TO_SHOOT;
                        }
                    } else {
                        done = false;
                        follower.followPath(paths.shootToPark(), 0.75, false);
                        pathState = PathState.PARK; }
                }
                break;

            case OUT:
                if (!follower.isBusy()) {
                    follower.followPath(paths.out(), 0.75, false);
                    if (!intake.haveBall()) {
                        pathState = PathState.IN;
                    } else {
                        pathState = PathState.TO_SHOOT;
                    }
                }
                break;

            case IN:
                if (!follower.isBusy()) {
                    follower.followPath(paths.in(), 0.75, false);
                    if (!intake.haveBall() && waitSecs(2)) {
                        pathState = PathState.TO_SHOOT;
                    } else if (intake.haveBall()) {
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

        shooter.setHood(hoodHeight);
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

        telemetry.addData("mirror", isMirror);
        telemetry.addData("alliance", alliance);
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new FarPaths(follower);

        isMirror = paths.bluePath(alliance);
        follower.setStartingPose(paths.startPose);

        if (isMirror) { turnTableAngle = -63; }
        shooter.rotateTurret(turnTableAngle);

        runtime.reset();
        pathState = PathState.START;
    }
}
