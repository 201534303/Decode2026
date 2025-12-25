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
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;


@TeleOp(name="Two Driver Tele", group="Iterative OpMode")
@Config
public class MainTeleOpBetter extends OpMode {

    private double DELETEBUTTHISISELAPSEDTIME1 = 0;
    private double DELETEBUTTHISISELAPSEDTIME2 = 0;

    //choose
    private Choose choose;

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

    private Choose.Alliance currentColor;

    @Override
    public void init() {
        //choose
        choose = new Choose(gamepad1, telemetry);


        //localization
        follower = Constants.createFollower(hardwareMap);

        //runtime
        overallRuntime = new ElapsedTime();

        //subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry, overallRuntime);

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry);
    }

    @Override
    public void init_loop() {
        currentColor = Choose.Alliance.RED;
        choose.allianceInit();
        currentColor = choose.getSelectedAlliance();
        telemetry.update();
    }

    @Override
    public void start() {
        overallRuntime.reset();
        if(currentColor == Choose.Alliance.RED){
            follower.setPose(new Pose(115, 70, 0));
        }
        if(currentColor == Choose.Alliance.BLUE){
            follower.setPose(new Pose(29, 70, Math.PI));
        }
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
        if (gamepad1.options){
            robot.setIMUZero(currentColor);
        }

        robot.fieldCentricDrive(currentColor);

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

        if(overallRuntime.time() - DELETEBUTTHISISELAPSEDTIME1 > 0.25){
            if(gamepad1.dpad_up){
                robot.shooter(25, 0);
                DELETEBUTTHISISELAPSEDTIME1 = overallRuntime.time();
            }
            if(gamepad1.dpad_down){
                robot.shooter(-25, 0);
                DELETEBUTTHISISELAPSEDTIME1 = overallRuntime.time();
            }
        }
        if(overallRuntime.time() - DELETEBUTTHISISELAPSEDTIME2 > 0.25){
            if(gamepad1.dpad_right){
                robot.shooter(0, 0.05);
                DELETEBUTTHISISELAPSEDTIME2 = overallRuntime.time();
            }
            if(gamepad1.dpad_left){
                robot.shooter(0, -0.05);
                DELETEBUTTHISISELAPSEDTIME2 = overallRuntime.time();
            }
        }

        /*
        --------------------------UPDATE--------------------------
         */

        robot.update(currentColor);
        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
