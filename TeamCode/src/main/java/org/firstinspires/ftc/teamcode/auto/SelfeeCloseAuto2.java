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
import org.firstinspires.ftc.teamcode.pedroPathing.Config.OLDConstants;
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
    private Drivetrain drivetrain;
    private Servo indicatorLight;
    private double turnTableAngle = 47;
    private double turnTableAngleFirst = 13;
    private double hoodHeight = 0.44498;//0.4;//

    //AUTO
    private Follower follower;
    private ClosePaths paths;
    private OLDChoose choose;
    private Timer actionTimer;
    private Timer overallTimer;
    private Timer timerTimer;
    private int spikeMark = 0;
    public enum PathState {
        START, TO_SHOOT, SHOOT, INTAKE, PARK, FIRST_SHOOT, TO_PARK
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

    private void clearNavigationFlags() {
        intakePathSet = false;
        toShootPathSet = false;
    }

    private boolean isCloseSelfeeCycle() {
        return spikeMark == 1 || spikeMark == 2 || (spikeMark == 3 && !fill);
    }

    private boolean isSecondLaneCycle() {
        return (spikeMark == 4 && !fill) || (spikeMark == 3 && fill);
    }

    private PathChain selectToShootPath() {
        if (spikeMark == 1) {
            return paths.ballCollect1ToShoot();
        }
        if (spikeMark == 2 || (spikeMark == 4 && !fill) || spikeMark == 3) {
            return paths.selfeeToShoot();
        }
        if (spikeMark == 4) {
            return paths._2ToShoot();
        }
        if (spikeMark == 5 && !fill) {
            return paths._2ToShoot2();
        }
        return paths._3ToShoot();
    }

    private PathChain selectIntakePath() {
        if (spikeMark == 0) {
            return paths.shootTo1();
        }
        if (spikeMark == 1 || spikeMark == 2) {
            return paths.shootToSelfee();
        }
        if (!fill && spikeMark == 3) {
            return paths.shootToSelfee2();
        }
        if (isSecondLaneCycle()) {
            return paths.shootTo2();
        }
        if (fill && spikeMark == 4) {
            return paths.shootTo3();
        }
        return paths.shootToPark();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                transitionTo(PathState.FIRST_SHOOT);
                break;

            case FIRST_SHOOT:
                if(!follower.isBusy() && !firstShootPathSet) {
                    firstShootPathSet = true;
                    follower.followPath(paths.firstToShoot(), 0.95, false);
                }

                if (follower.atParametricEnd() && firstShootPathSet) {
                    doneOne = true;
                    firstShootPathSet = false;
                    transitionTo(PathState.SHOOT);
                }
                break;

            case TO_SHOOT:
                shooter.setHood(hoodHeight);

                if (!follower.isBusy() && !toShootPathSet) {
                    toShootPathSet = true;
                    follower.followPath(selectToShootPath(), 0.95, true);
                }
                if (toShootPathSet) {
                    if(spikeMark == 4){
                        shootMove = true;
                        if(y > x){
                            intake.allTheWay();
                        }
                    } else if (spikeMark >= 2 && waitSecs(0.75)) {
                        intake.setIntakeSpeed(0.3);
                    }
                    if (follower.atParametricEnd()) {
                        toShootPathSet = false;
                        shootMove = false;
                        transitionTo(PathState.SHOOT);
                    }
                }
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    intake.allTheWay();
                    if (waitSecs(0.6)) {//0.6
                        clearNavigationFlags();
                        transitionTo(PathState.INTAKE);
                    }
                }
                break;

            case INTAKE:
                if(intake.haveBall() && waitSecs(0.75)){
                    spikeMark += 1;
                    intakePathSet = false;
                    transitionTo(PathState.TO_SHOOT);
                    break;
                }

                if (!follower.isBusy()) {
                    intake.transferOff();
                    intake.intakeIn();

                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(selectIntakePath(), spikeMark == 5 ? 0.6 : 0.95, true);
                    }

                    if(isCloseSelfeeCycle() && intakePathSet && follower.atParametricEnd()){
                        if(onceTimer){
                            timerTimer.resetTimer();
                            onceTimer = false;
                        }
                        if(timerTimer.getElapsedTimeSeconds() >= 3) {
                            drivetrain.driveRobot(0, 0, 0);
                        } else {
                            drivetrain.driveRobotHeading(0.57, Math.toRadians(35), heading);
                        }

                    }

                    if (spikeMark == 0 && waitForPathEndOrTimeout(1, 2)) {
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    } else if (isCloseSelfeeCycle() && waitForPathEndOrTimeout(3.75, 4)) {
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    } else if (isSecondLaneCycle() && waitForPathEndOrTimeout(0.5, 1)) {
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    } else if (spikeMark == 4 && fill && waitForPathEndOrTimeout(3.5, 4)){
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    }
                    else if (spikeMark == 5 && follower.atParametricEnd()) {
                        intakePathSet = false;
                        transitionTo(PathState.PARK);
                    }
                }
                break;

            case TO_PARK:
                intake.off();
                done = true;

                if (!didParking) {
                    follower.followPath(paths._ToPark(), 0.6, true);
                    didParking = true;
                    pathState = PARK;
                }
                break;

            case PARK:
                if(!parkActions) {
                    parkActions = true;
                    done = true;
                    shooter.rotateTurret(0);
                    intake.transferOff();
                    intake.off();
                    shooter.off();
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
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        indicatorLight = hardwareMap.get(Servo.class, "taillight");

        shooter.setHood(0.60);
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
            turnTableAngleFirst = -5;
            turnTableAngle = -45;
        } else {
            turnTableAngle = 47;
            turnTableAngleFirst = 13;
        }

        shooter.rotateTurret(turnTableAngleFirst);

        telemetry.update();
    }

    public void start() {
        follower = OLDConstants.createFollower(hardwareMap);
        paths = new ClosePaths(follower);

        isMirror = paths.bluePath(alliance);//mirrors the paths if blue
        follower.setStartingPose(paths.startPose);//sets up the starting pose

        if(isMirror) {
            turnTableAngleFirst = -9;
            turnTableAngle = -45;
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

            double virtualX = x + time*vel.getXComponent();
            double virtualY = y + time*vel.getYComponent();

            robotActions.updateShooter(alliance, virtualX, virtualY, 0);

        } else if (!done && doneOne) {
            robotActions.updateTurret(alliance, x, y, heading);
            shooter.close();
        } else {
            shooter.close();
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
