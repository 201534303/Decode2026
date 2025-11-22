package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;

@TeleOp(name="mainTeleOp", group="Iterative OpMode")
@Config
public class MainTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private IntakeTele intake;
    private ShooterTele shooter;
    private Telemetry dash;
    public static double power;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
        intake = new IntakeTele(hardwareMap, gamepad1, gamepad2, telemetry);
        shooter = new ShooterTele(hardwareMap, gamepad1, gamepad2, telemetry, runtime);
        telemetry.addData("Status", "Initialized");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //driving
        dt.feildCentricDrive();
        dt.printData();

        //intake
        intake.update();

        //shooter
        shooter.update();

        //update power var
        power = shooter.power;

        //telemetry
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
