package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;

@TeleOp(name="mainTeleOp", group="Iterative OpMode")
@Config
public class MainTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private IntakeTele intake;
    private Follower follower;

    ShooterTele shooter;
    private Telemetry dash;
    public static double power;
    public static double targetVelo = 1300;




    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
        intake = new IntakeTele(hardwareMap, gamepad1, gamepad2, telemetry);
        shooter = new ShooterTele(hardwareMap, gamepad1, gamepad2, telemetry, runtime);


        follower = Constants.createFollower(hardwareMap);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
        //dt.resetHeading();
    }

    @Override
    public void loop() {

        //driving

        follower.update();

        dt.feildCentricDrive(Math.toDegrees(follower.getPose().getHeading()));
        //dt.updateOdo();

        //intake
        intake.update();

        //shooter

        if(gamepad2.dpad_right){
            shooter.setMode(0);
        }

        if(gamepad2.dpad_left){
            shooter.setMode(1);
        }

        if(shooter.getMod() == 0){
            //shooter.setTurretAngle(-(90-Math.toDegrees(Math.atan((144-Math.abs(follower.getPose().getY()))/follower.getPose().getX()))));
        }

        else if(shooter.getMod() == 1){
            if(gamepad2.dpad_up){
                //shooter.setTurretAngle(0);
            }

            else if (gamepad2.right_bumper){
                if(shooter.getTurrentAngle() < 80){
                    //shooter.setTurretAngle(shooter.getTurrentAngle()+1);
                }
            }

            else if (gamepad2.left_bumper){
                if(shooter.getTurrentAngle() > -80){
                    //shooter.setTurretAngle(shooter.getTurrentAngle()-1);
                }
            }

        }

        shooter.shooterMachine();

        //update power var
        power = shooter.power;

        //telemetry



        dash.addData("shooter vel", shooter.getMotorVel());
        dash.addData("target vel", targetVelo);
        dash.addData("flywheel rpm", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
    }

    @Override
    public void stop() {
    }
}
