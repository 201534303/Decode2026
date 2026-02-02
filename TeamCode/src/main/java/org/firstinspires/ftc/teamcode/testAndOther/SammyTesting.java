package org.firstinspires.ftc.teamcode.testAndOther;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.ShooterTele;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

@Config
@TeleOp(name = "Sammy Shooter Testing")

public class SammyTesting extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Servo hood;
    Shooter shooter;
    private Telemetry telemetry;

    public static double x;
    public static double g;
    public static double a;
    public static double highH;
    public static double lowH;
    private double sammyEquation (double x, double g, double a, double highH, double lowH) {
        double s1 = Math.cos(a);
        double s2 = 2 * Math.pow(s1, 2);
        double s3 = (x * Math.tan(a)) - (highH - lowH);
        double s4 = s2 * s3;
        double s5 = g / s4;
        return x * s5;

        //delta h hight of
        //x horizantal direct distance to the goal 
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        hood = hardwareMap.get(Servo.class, "hood");
        shooter = new Shooter(hardwareMap, telemetry, runtime);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        double flywheelSpeed = sammyEquation(x, g, a, highH, lowH);
        telemetry.addData("equation:", flywheelSpeed);
        telemetry.addData("flywheel speed (rpm):", shooter.getMotorVel());
        telemetry.addData("hood height:", hood.getPosition());
    }
}
