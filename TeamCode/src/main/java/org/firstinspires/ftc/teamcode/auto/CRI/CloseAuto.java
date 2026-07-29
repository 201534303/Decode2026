package org.firstinspires.ftc.teamcode.auto.CRI;

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
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.ClosePaths;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;

@Autonomous(name = "CloseAuto")

public class CloseAuto extends OpMode {
    private static final double AUTO_BLUE_LIGHT = 0.63;
    private static final double AUTO_RED_LIGHT = 0.28;

    //ROBOT
    private IntakeAuto intake;
    private ShooterAuto shooter;
    private RobotActions robotActions;
    private Drivetrain drivetrain1;
    private Servo indicatorLight;
    private double turnTableAngleFirst = 15;

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
    private boolean parkActions = false;
    private boolean done = false;
    private boolean firstShootPathSet = false;
    private boolean toShootPathSet = false;
    private boolean intakePathSet = false;
    private boolean shootMove;
    private int allianceOffset = 10;

    private double x, y, heading;

    public void resetActionTimer(){ actionTimer.resetTimer(); }//resets timer
    public boolean waitSecs(double seconds){ return actionTimer.getElapsedTimeSeconds() > seconds; }

    private void transitionTo(PathState newState) {
        resetActionTimer();
        pathState = newState;
    }

    private boolean waitForPathEndOrTimeout(double timeoutSecs) {
        return (follower.atParametricEnd() || waitSecs(timeoutSecs));
    }

    private void clearNavigationFlags() {
        intakePathSet = false;
        toShootPathSet = false;
    }

    private PathChain selectToShootPath() {
        if (spikeMark == 1) {
            return paths.ballCollect1ToShoot();
        }
        if (spikeMark == 2 || spikeMark == 3 || spikeMark == 4 || spikeMark == 5) {
            return paths.selfeeToShootLine();
        }
        if(spikeMark == 6 && !fill){
            return paths.sefeeToShoot2();
        }
        return paths._3ToShootPt2();
    }

    private PathChain selectIntakePath() {
        if (spikeMark == 0) {
            return paths.shootTo1();
        }
        if (spikeMark == 1 || spikeMark == 2 || spikeMark == 3 || spikeMark == 4 || spikeMark == 5) {
            return paths.shootToSelfeeLine();
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

                if(waitSecs(1.20)){ // 1.25
                    intake.allTheWay();
                }

                if (follower.atPose(paths.shootPose0, 5, 5)) {
                    if(alliance == OLDChoose.Alliance.RED) {
                        shooter.rotateTurret(15);
                    } else{
                        shooter.rotateTurret(-15);
                    }
                    firstShootPathSet = false;
                    transitionTo(PathState.SHOOT);
                }
                break;

            case TO_SHOOT:
                shootMove = spikeMark == 6;

                if(!shootMove) {
                    if(spikeMark == 1) {
                        shooter.setHood(0.7); // 0.8
                    } else{
                        shooter.setHood(0.834886007091);
                    }
                }

                if (!toShootPathSet) {
                    toShootPathSet = true;
                    follower.followPath(selectToShootPath(), 1, false);
                }
                if (toShootPathSet) {
                    if(spikeMark == 6){
                        if(follower.atPose(paths.shootPose2, 20, 20)) {
                            toShootPathSet = false;
                            transitionTo(PathState.SHOOT);
                        }
                    } else if (follower.atPose(paths.shootPose, 5, 5)) {
                        toShootPathSet = false;
                        transitionTo(PathState.SHOOT);
                    }
                }
                break;

            case SHOOT:
                intake.allTheWay();

                if(spikeMark == 6){
                    if(waitSecs(0.4)){
                        shootMove = false;
                        doneOne = false;

                        if(alliance == OLDChoose.Alliance.RED) {
                            shooter.rotateTurret(25);
                        } else{
                            shooter.rotateTurret(-25);
                        }
                    }
                }

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
                if(waitSecs(0.2)) {
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

                if(!(spikeMark == 0)) {
                    doneOne = true;
                }

                if(spikeMark == 0){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(selectIntakePath(), 1, false);
                    }
                }

                if(spikeMark == 0 && waitSecs(1.4)){
                    doneOne = false;

                    if(alliance == OLDChoose.Alliance.RED) {
                        shooter.rotateTurret(50);
                    } else{
                        shooter.rotateTurret(-50);
                    }
                }

                if (spikeMark == 0 && waitForPathEndOrTimeout(2.5)) {
                    spikeMark += 1;
                    shootMove = false;
                    intakePathSet = false;
                    transitionTo(PathState.TO_SHOOT);
                } else if(spikeMark == 1 || spikeMark == 2 || (spikeMark == 5 && !fill) || spikeMark == 3 || spikeMark == 4){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(selectIntakePath(), 1, true);
                    } else if(waitForPathEndOrTimeout(2.9)){

                        if(spikeMark == 5){
                            if(waitForPathEndOrTimeout(2.95)){
                                if (alliance == OLDChoose.Alliance.RED) {
                                    drivetrain1.driveRobotHeadingAndLine(.3, Math.toRadians(30), heading, x, y, 130, 53);
                                } else {
                                    drivetrain1.driveRobotHeadingAndLine(.3, Math.toRadians(150), heading, x, y, 11.5, 53);
                                }

                                if (waitSecs(3)) {
                                    spikeMark += 1;
                                    intakePathSet = false;
                                    transitionTo(PathState.TO_SHOOT);
                                }
                            }
                        } else {

                            if (alliance == OLDChoose.Alliance.RED) {
                                drivetrain1.driveRobotHeadingAndLine(.3, Math.toRadians(30), heading, x, y, 130, 53);
                            } else {
                                drivetrain1.driveRobotHeadingAndLine(.3, Math.toRadians(150), heading, x, y, 11.5, 53);
                            }

                            if (waitSecs(2.9)) {
                                spikeMark += 1;
                                intakePathSet = false;
                                transitionTo(PathState.TO_SHOOT);
                            }
                        }
                    }
                } else if(spikeMark == 5){
                    if (!intakePathSet) {
                        intakePathSet = true;
                        follower.followPath(paths.shootTo3(), 1, false);
                    } else if(waitForPathEndOrTimeout(3)){
                        doneOne = false;

                        if(alliance == OLDChoose.Alliance.RED) {
                            shooter.rotateTurret(48);
                        } else{
                            shooter.rotateTurret(-43); // change here for third spike angle
                        }
                        spikeMark += 1;
                        intakePathSet = false;
                        transitionTo(PathState.TO_SHOOT);
                    }
                }
                else if (spikeMark == 6){
                    spikeMark += 1;
                    intakePathSet = false;
                    transitionTo(PathState.PARK);
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

        shooter.setHood(0.45); // 0.6
    }

    public void init_loop(){
        if(!readyAlliance) {
            alliance = choose.getSelectedAlliance();
            readyAlliance = choose.allianceInit();
        } else {
            fill = choose.getFill();
            choose.fillInit();
        }

        isMirror = alliance == OLDChoose.Alliance.BLUE;
        indicatorLight.setPosition(isMirror ? AUTO_BLUE_LIGHT : AUTO_RED_LIGHT);

        if (isMirror) {
            turnTableAngleFirst = -11;
            allianceOffset = 10; // 0
        } else {
            turnTableAngleFirst = 14;
            allianceOffset = -10; // red
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
            turnTableAngleFirst = -11;
            allianceOffset = 10; // 0
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
            robotActions.updateTurret(alliance, (x + allianceOffset), y, heading);
            shooter.close();
        }
        if (!doneOne && spikeMark == 0) {
            shooter.closeFaster();
        } else if ((last && !shootMove) || (spikeMark <= 6 && !doneOne)){
            shooter.set(1150);
        }
        follower.update();

        autonomousPathUpdate();//main auto code

        telemetry.addData("mirror", isMirror);
        telemetry.addData("path state", pathState);
        telemetry.addData("spike mark", spikeMark);
        telemetry.addData("alliance", alliance);
        telemetry.addData("runtime", overallTimer);

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
