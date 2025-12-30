package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous(name = "FarAuto")

public class FarAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private FarPaths paths;

    private IntakeAuto intake;
    private ShooterAuto shooter;
    private Choose choose;
    private ElapsedTime runtime = new ElapsedTime();

    private int spikeMark = 1;
    private Choose.Alliance alliance = Choose.Alliance.RED;
    private boolean once = false;
    private boolean done = false;

    public enum PathState {
        COLLECT_SHOOT, SHOOT_COLLECT, SHOOT,
        OFF, START, IN, OUT
    }
    private boolean isMirror = false;
    private double turnTableAngle = 58;
    PathState pathState = PathState.START;

    public void resetActionTimer(){ actionTimer.resetTimer(); }
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START://start state
                resetActionTimer();
                follower.followPath(paths.startToShoot(), 0.75, false);
                pathState = PathState.SHOOT;
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    shooter.rotateTurret(turnTableAngle);
                    if (waitSecs(2)){
                        intake.allTheWaySlow();//go all the way to shoot
                        resetActionTimer();
                        pathState = PathState.SHOOT_COLLECT;
                    }
                }
                break;

            case SHOOT_COLLECT:
                if (!follower.isBusy() && waitSecs(1.25)) {//waits 0.5 works
                    intake.intakeIn();
                    intake.transferOff();

                    if (spikeMark == 2) {
                        follower.followPath(paths.shootToIn(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.OUT;
                    } else if (spikeMark == 3) {
                        follower.followPath(paths.shootToSide(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.OUT;
                    }else if (spikeMark < 4){
                        follower.followPath(paths.shootTo(), 0.75, false);
                        resetActionTimer();
                        pathState = PathState.OUT;
                    }
                    else { pathState = PathState.OFF; }
                }
                break;

            case COLLECT_SHOOT:
                if (!follower.isBusy()  &&  waitSecs(1) || waitSecs(2)) {
                    intake.transferOff();
                    intake.setIntakeSpeed(0.5);
                    if (spikeMark < 4) {
                        follower.followPath(paths.collectToShoot(), 0.8, true);
                        spikeMark++;
                        resetActionTimer();
                        pathState = PathState.SHOOT;
                    } else{ pathState = PathState.OFF; }
                }
                break;


            case OUT:
                if(!follower.isBusy() && waitSecs(1.5) || waitSecs(2)){
                    //intake.setIntakePower(-0.5);
                    if (spikeMark == 3){
                        follower.followPath(paths.OutSide(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.IN;
                    } else if (spikeMark == 2){
                        follower.followPath(paths.OutSideIn(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.IN;
                    } else {
                        follower.followPath(paths.Out(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.IN;
                    }
                }
                break;

            case IN:
                if(!follower.isBusy() && waitSecs(1)){
                    intake.intakeIn();
                    if (spikeMark == 1){
                        follower.followPath(paths.In(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    } else if (spikeMark == 2){
                        follower.followPath(paths.InSideIn(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                        //once = true;
                    } else {
                        follower.followPath(paths.InSide(), 0.75, true);
                        resetActionTimer();
                        pathState = PathState.COLLECT_SHOOT;
                    }
                }
                break;

            case OFF:
                if(!follower.isBusy()) {
                    follower.followPath(paths.shootToOut(), 0.8, true);
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
        pathTimer = new Timer();
        actionTimer = new Timer();

        choose = new Choose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);

        shooter.setHood(0.2);
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
        telemetry.addData("spikeMark", spikeMark);
        telemetry.addData("path state", pathState);
        telemetry.addData("alliance", alliance);
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

        if (isMirror) { turnTableAngle = turnTableAngle * -1; }
        shooter.rotateTurret(turnTableAngle);

        runtime.reset();
        pathTimer.resetTimer();
        pathState = PathState.START;
    }
}
