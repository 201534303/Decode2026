package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="drive and intake", group="Iterative OpMode")
public class MainTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private Intake intake;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
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
    }

    @Override
    public void stop() {
    }
}
