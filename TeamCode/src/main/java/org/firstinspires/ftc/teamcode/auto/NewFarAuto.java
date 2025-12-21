package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystems.Auto.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Auto.ShooterAuto;

@Autonomous
public class NewFarAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;
    private FarPaths paths;

    //robot stuff
    private IntakeAuto intake;
    private DrivetrainAuto drivetrain;
    private ShooterAuto shooter;
    private ElapsedTime runtime = new ElapsedTime();
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.allTheWay();
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    if(waitSecs(1)){
                        intake.transferOff();
                        follower.followPath(paths.shootTo1(), 0.6, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.collectToShoot(), 0.6, true);
                    setPathState(3);
                }
                break;

            case 3:
                intake.allTheWay();
                setPathState(4);
                break;
        }
    }

    public void setUp(){
        //shooter.rotateTurret(-50);
        shooter.hoodPitch(0.75);
    }

    public boolean waitSecs(double seconds){
        return actionTimer.getElapsedTimeSeconds() > seconds;
    }

    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.resetTimer();
    }

    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        paths = new FarPaths(follower);
        follower.setStartingPose(paths.startPose);
        intake = new IntakeAuto(hardwareMap, telemetry);
        shooter = new ShooterAuto(hardwareMap, telemetry, runtime);
        drivetrain = new DrivetrainAuto(hardwareMap, telemetry);

        setUp();
    }

    public void loop() {
        shooter.close();
        follower.update();

        autonomousPathUpdate();//main auto code

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("flywheel RPM", shooter.getMotorRPM());
        telemetry.update();
    }

    public void start() {
        runtime.reset();
        pathTimer.resetTimer();
        setPathState(0);
    }
}
