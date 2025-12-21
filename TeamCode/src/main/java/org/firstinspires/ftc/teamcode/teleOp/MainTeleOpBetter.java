package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

@TeleOp(name="mainTeleOpBetter", group="Iterative OpMode")
@Config
public class MainTeleOpBetter extends OpMode {

    //runtime
    private ElapsedTime overallRuntime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    //localization
    private Follower follower;

    //robot
    private RobotActions robot;

    //color
    public enum color{
        RED,
        BLUE
    }

    private color currentColor;

    @Override
    public void init() {

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);

        //localazation
        follower = Constants.createFollower(hardwareMap);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry);
    }

    @Override
    public void init_loop() {
        currentColor = color.RED;
        switch (currentColor){
            case RED:
                telemetry.addData("Color", " red");
                if(gamepad1.y){
                    currentColor = color.BLUE;
                }
                break;
            case BLUE:
                telemetry.addData("Color", " blue");
                if(gamepad1.y){
                    currentColor = color.RED;
                }
                break;
        }
    }

    @Override
    public void start() {
        overallRuntime.reset();
    }

    @Override
    public void loop() {

        /*
        --------------------------DRIVER ONE CONTROLS--------------------------
         */

        //reset localization to back
        if (gamepad1.share){
            robot.setLocalizationBack();
        }

        //reset imu to 0
        if (gamepad1.share){
            robot.setIMUZero();
        }

        robot.fieldCentricDrive();

        /*
        --------------------------DRIVER TWO CONTROLS--------------------------
         */

        robot.updateIntake();
        robot.updateTransfer();

        //shooting
        if(gamepad2.y){
            robot.setShootingAuto();
        }
        if(gamepad2.a){
            robot.setShootingOff();
        }

        /*
        --------------------------OTHER--------------------------
         */

        robot.setTurret(currentColor);

        /*
        --------------------------UPDATE--------------------------
         */
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
