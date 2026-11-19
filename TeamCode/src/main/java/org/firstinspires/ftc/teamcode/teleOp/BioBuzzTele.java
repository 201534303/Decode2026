package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.auto.util.PoseSaver;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class BioBuzzTele extends OpMode {

    //choose
    private OLDChoose choose;

    //runtime
    private ElapsedTime overallRuntime;
    private double lastTime;

    //subsystems
    private Drivetrain drivetrain;
    private Kicker kicker;
    private Intake intake;
    private Shooter shooter;
    private Feedback feedback;

    //localization
    private Follower follower;

    //robot
    private RobotActions robot;
    private OLDChoose.Alliance currentColor = RED;
    private double x;
    private double y;
    private double heading;
    private Vector vel;
    private double counter = 0;
    private double timeSinceLastLochalazationReset = 0;
    private boolean moving;
    private boolean rotating;
    private boolean movingOrRotating;
    LimelightProcessor_v3Tele ll;
    private Telemetry dash;
    public static double kf = 0.59;
    private double timeDif = 1.0;
    private double oldHeading = 0;
    private boolean kickerDown;

    private double shooterSpeed = 1360;

    //89.7,5.6
    @Override
    public void init() {
        ll = new LimelightProcessor_v3Tele(hardwareMap);
        //choose
        choose = new OLDChoose(gamepad1, telemetry);


        //localization
        follower = Constants.createFollower(hardwareMap);

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, overallRuntime);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);
        feedback = new Feedback(hardwareMap, overallRuntime, telemetry, gamepad1, gamepad2);
        kicker = new Kicker(hardwareMap, telemetry, overallRuntime);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry, feedback);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
        kicker.kickUp();
    }

    @Override
    public void init_loop() {}

    @Override

    public void start() {}

    @Override
    public void loop() {
        double nowTime = overallRuntime.time(TimeUnit.MILLISECONDS);
        timeDif = nowTime - lastTime;
        lastTime = nowTime;
        telemetry.addData("loop time", timeDif);

        Pose robotPos = follower.getPose();
        x = robotPos.getX();
        y = robotPos.getY();
        heading = robotPos.getHeading();

        /*
        --------------------------DRIVER ONE CONTROLS--------------------------
         */

        //reset imu to 0
        if (gamepad1.options){
            robot.setIMUZero(x, y, currentColor);
        }

        //drive
        if(!kickerDown) {
            robot.fieldCentricDrive(currentColor, heading, vel);
        }

        if(gamepad1.dpadUpWasPressed()){
            shooterSpeed += 10;
        }

        if(gamepad1.dpadDownWasPressed()){
            shooterSpeed -= 10;
        }

        /*
        --------------------------DRIVER TWO CONTROLS--------------------------
         */

        if(gamepad2.dpadDownWasPressed()){
            robot.toggleLiftMode();
        }

        if(gamepad2.yWasPressed() && gamepad2.right_trigger_pressed){
            kicker.kickDown();
            kickerDown = true;
        } else if(gamepad2.aWasPressed()){
            kicker.kickUp();
            kickerDown = false;
        }

        if(kickerDown){
            drivetrain.setMotorPowers(0,0,0,0);
            shooter.setVel(0);
        }

        telemetry.addData("alliance Color", currentColor);
        telemetry.addData("shooter speed", shooterSpeed);
        telemetry.addData("position", "(" + Math.round(x*100)/100.0 + "," + Math.round(y*100)/100.0 + ") Heading: " + Math.round(heading*10000)/10000.0);
        telemetry.addData("dist", Math.round(100.0*Math.hypot(144-x, 144-y))/100.0);

        robot.updateBioBuzz(currentColor, x, y, vel, shooterSpeed);
        if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1 )
            robot.updateIntake();
        follower.update();


        feedback.updateBallState(intake.haveBall());
        telemetry.update();
        dash.update();

        movingOrRotating = moving || rotating;
        oldHeading = heading;
    }

    @Override
    public void stop() {
    }
}

