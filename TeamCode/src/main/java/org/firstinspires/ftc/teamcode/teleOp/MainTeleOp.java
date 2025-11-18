package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;

@TeleOp(name="mainTeleOp", group="Iterative OpMode")
public class MainTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private Intake intake;
    private ShooterTele shooter;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
        intake = new Intake(hardwareMap);
        shooter = new ShooterTele(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
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
        dt.feildCentricDrive();
        dt.printData();
        intake.intakeIn();
        telemetry.update();
        if(gamepad1.a){
            shooter.setPower(.4);
        }
        if(gamepad1.b){
            shooter.setPower(0);
        }
    }

    @Override
    public void stop() {
    }
}
