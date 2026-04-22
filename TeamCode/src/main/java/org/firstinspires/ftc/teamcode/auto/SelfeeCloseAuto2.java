package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.SelfeeCloseAuto2.PathState.PARK;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;

@Autonomous(name = "SelfeeCloseAuto")

public class SelfeeCloseAuto2 extends OpMode {
    private static final double AUTO_BLUE_LIGHT = 0.63;
    private static final double AUTO_RED_LIGHT = 0.28;

    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private RobotActions robotActions;
    private Drivetrain drivetrain1;
    private Servo indicatorLight;
    private double turnTableAngleFirst = 15;
    private double hoodHeight = 0.834886007091;//0.4;//

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private Timer overallTimer;
    private Timer timerTimer;
    private int spikeMark = 0;
    private boolean last;
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK, FIRST_SHOOT
    }
    PathState pathState = PathState.START;
    private OLDChoose.Alliance alliance = OLDChoose.Alliance.RED;
    private boolean fill = false;
    private boolean doneOne = false;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMirror = false;
    private boolean readyAlliance = false;
    private boolean readyFill = false;
    private boolean parkActions = false;
    private boolean done = false;
    private boolean firstShootPathSet = false;
    private boolean toShootPathSet = false;
    private boolean intakePathSet = false;
    private boolean didParking = false;
    private boolean shootMove;
    private boolean onceTimer;

    private double x, y, heading;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    private void transitionTo(PathState newState) {
        resetActionTimer();
        pathState = newState;
    }

    private boolean waitForPathEndOrTimeout(double endDelaySecs, double timeoutSecs) {
        return (follower.atParametricEnd() && waitSecs(endDelaySecs)) || waitSecs(timeoutSecs);
    }

    private boolean waitForPathEndAndTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() && waitSecs(timeoutSecs));
    }

    private boolean waitForPathEndOrTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() || waitSecs(timeoutSecs));
    }

    private void clearNavigationFlags() {
        intakePathSet = false;
        toShootPathSet = false;
    }

    private boolean isCloseSelfeeCycle() {
        return spikeMark == 1 || spikeMark == 2 || spikeMark == 4 || spikeMark == 3;
    }

    private PathChain selectToShootPath() {
        if (spikeMark == 1) {
            return paths.ballCollect1ToShoot();
        }
        if (spikeMark == 2 || spikeMark == 3 || (spikeMark == 5 && !fill) || spikeMark == 4) {
            return paths.selfeeToShoot();
        }
        if (spikeMark == 6) {
            return paths._2ToShoot2();
        }
        return paths._3ToShoot();
    }

    private PathChain selectIntakePath() {
        if (spikeMark == 0) {
            return paths.shootTo1();
        }
        if (spikeMark == 1 || spikeMark == 2 || spikeMark == 4 || spikeMark == 3) {
            return paths.shootToSelfee();
        }
        if (spikeMark == 5) {
            return paths.shootTo2();
        }
        return paths.shootToPark();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                transitionTo(PathState.FIRST_SHOOT);
                break;

            case FIRST_SHOOT:
                if(waitSecs(0.3) && !firstShootPathSet){//0.3
                    firstShootPathSet = true;
                    shootMove = false;
                    follower.followPath(paths.firstToShoot(), 1, false);
                }

                if(waitSecs(1.25)){
                    intake.allTheWay();
                }

                if (follower.atPose(paths.shootPose0, 5, 5)) {
                    shooter.rotateTurret(20);
                    firstShootPathSet = false;
                    transitionTo(PathState.SHOOT);
                }
                break;

            case TO_SHOOT:
                shootMove = spikeMark == 6;
                if(!shootMove) {
                    shooter.setHood(hoodHeight);
                }

                if (!toShootPathSet) {
                    toShootPathSet = true;
                    follower.followPath(selectToShootPath(), 1, false);
                }
                if (toShootPathSet) {
                    if(spikeMark == 6){
                        if(y > x){
                            intake.allTheWay();
                        }
                    } else if (spikeMark >= 2 && waitSecs(0.75)) {
                        intake.setIntakeSpeed(0.3);
                    }

                    if(spikeMark == 6){
                        if(follower.atPose(paths.shootPose2, 20, 20)) {
                            toShootPathSet = false;
                            transitionTo(PathState.SHOOT);
                        }
                    } else if (spikeMark == 1 && follower.atPose(paths.shootPose, 5, 5)) {
                        toShootPathSet = false;
                        transitionTo(PathState.SHOOT);
                    }
                    else if (!(spikeMark == 1) && follower.atPose(paths.shootPose, 10, 10)) {
                        toShootPathSet = false;
                        transitionTo(PathState.SHOOT);
                    }
                }
                break;

            case SHOOT:
                intake.allTheWay();

                if(spikeMark == 0){
                    clearNavigationFlags();
                    transitionTo(PathState.INTAKE);
                } else {
                    intake.allTheWay();
                    if(spikeMark == 1){
                        if (waitSecs(0.65)) {//0.6
                            clearNavigationFlags();
                            transitionTo(PathState.INTAKE);
                        }
                    } else {
                        if (waitSecs(0.55)) {//0.6
                            clearNavigationFlags();
                            transitionTo(PathState.INTAKE);
                        }
                    }
                }
                break;

            case INTAKE:
                if(waitSecs(1)) {
                    intake.transferOff();
                    intake.intakeIn();
                }

                if(intake.haveBall() && waitSecs(1)){
                    if(spikeMark != 0) {
                        doneOne = true;
                    }
                    spikeMark += 1;
                    intakePathSet = false;
                    transitionTo(PathState.TO_SHOOT);
                    break;
                }

                if(! (spikeMark == 0)) {
                    doneOne = true;
                }

                if(spikeMark == 0 || spikeMark == 5){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(selectIntakePath(), 1, false);
                    }
                }

                if(spikeMark == 0 && waitSecs(1.5)){
                    doneOne = false;
                    shooter.rotateTurret(40);
                }

                if (spikeMark == 0 && waitForPathEndOrTimeout(2.5)) {
                    spikeMark += 1;
                    //doneOne = true;
                    shootMove = false;
                    intakePathSet = false;
                    transitionTo(PathState.TO_SHOOT);
                } else if (spikeMark == 5 && waitForPathEndOrTimeout(2.75)) {
                    shooter.rotateTurret(30);
                    shooter.setHood(1);
                    last = true;
                    spikeMark += 1;
                    intakePathSet = false;
                    transitionTo(PathState.TO_SHOOT);
                } else if(spikeMark == 1 || spikeMark == 2 || (spikeMark == 4 && !fill) || spikeMark == 3){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(selectIntakePath(), 1, true);
                    } else if(waitForPathEndOrTimeout(2.5)){
                        drivetrain1.driveRobotHeadingAndLine(.3, Math.toRadians(30), heading, x, y, 130,53);
                        if(waitSecs(3)) {
                            spikeMark += 1;
                            intakePathSet = false;
                            transitionTo(PathState.TO_SHOOT);
                        }
                    }
                } else if(spikeMark == 4 && fill){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(paths.shootTo3(), 1, false);
                    } else if(waitForPathEndOrTimeout(3)){
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    }
                }
                else if (spikeMark == 6){
                    //shootMove = false;
                    spikeMark += 1;
                    intakePathSet = false;
                    transitionTo(PARK);
                }
                break;

            case PARK:
                if(!parkActions) {
                    if(waitSecs(0.5)) {
                        parkActions = true;
                        done = true;
                        shootMove = false;
                        shooter.rotateTurret(0);
                        intake.transferOff();
                        intake.off();
                        shooter.off();
                    }
                }
                break;
        }
    }

    public void init() {
        actionTimer = new Timer();
        overallTimer = new Timer();
        timerTimer = new Timer();

        choose = new OLDChoose(gamepad1, telemetry);
        intake = new IntakeAuto(hardwareMap, telemetry, runtime);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        robotActions = new RobotActions(shooter, follower, telemetry);
        drivetrain1 = new Drivetrain(hardwareMap, telemetry);
        indicatorLight = hardwareMap.get(Servo.class, "taillight");

        shooter.setHood(0.40); // 0.6
    }

    public void init_loop(){
        if(!readyAlliance) {
            alliance = choose.getSelectedAlliance();
            readyAlliance = choose.allianceInit();
        } else {
            fill = choose.getFill();
            readyFill = choose.fillInit();
        }

        if(alliance == OLDChoose.Alliance.BLUE){
            isMirror = true;
        } else {
            isMirror = false;
        }
        indicatorLight.setPosition(isMirror ? AUTO_BLUE_LIGHT : AUTO_RED_LIGHT);

        if (isMirror) {
            turnTableAngleFirst = -9;
        } else {
            turnTableAngleFirst = 11;
        }

        shooter.rotateTurret(turnTableAngleFirst);

        telemetry.update();
    }

    public void start() {
        follower = Constants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);

        isMirror = paths.bluePath(alliance);//mirrors the paths if blue
        follower.setStartingPose(paths.startPose);//sets up the starting pose

        if(isMirror) {
            turnTableAngleFirst = -9;
        }//if it's mirrored turn the turntable

        shooter.rotateTurret(turnTableAngleFirst);

        runtime.reset();//resets overall timer
        overallTimer.resetTimer();
        timerTimer.resetTimer();
        actionTimer.resetTimer();
        pathState = PathState.START;//sets the path state
    }

    public void loop() {
        Pose position = follower.getPose();
        x = position.getX();
        y = position.getY();
        heading = position.getHeading();

        if (shootMove){
            Vector vel = follower.getVelocity();

            double time = robotActions.calculateIterativeLeadTime(alliance, x, y, vel);

            double velX = vel != null ? vel.getXComponent() : 0;
            double velY = vel != null ? vel.getYComponent() : 0;
            double robotSpeed = vel != null ? vel.getMagnitude() : 0;
            double virtualX = x + time*velX;
            double virtualY = y + time*velY;

            robotActions.updateTurret(alliance, virtualX, virtualY, heading);
            robotActions.updateShooter(alliance, virtualX, virtualY, robotSpeed);

        } else if (!done && doneOne && !last) {
            robotActions.updateTurret(alliance, x, y, heading);
            shooter.close();
        }
        if (!doneOne) {
            shooter.closeFaster();
        } else if (last && !shootMove){
            shooter.set(1150);
        }
        follower.update();

        autonomousPathUpdate();//main auto code

        //telemetry.addData("haveBall", intake.haveBall());
        telemetry.addData("mirror", isMirror);
        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("alliance", alliance);
        telemetry.addData("runtime", overallTimer);

//        if(overallTimer.getElapsedTimeSeconds() > 29){
//            pathState = TO_PARK;
//        }

        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());
        //telemetry.addData("heading", follower.getPose().getHeading());
        //telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    @Override
    public void stop(){
        for (int i  = 0; i < 50; i++){
            Pose p = follower.getPose();
            PoseSaver.save(p.getX(), p.getY(), p.getHeading());
            follower.update();
        }
    }
}
