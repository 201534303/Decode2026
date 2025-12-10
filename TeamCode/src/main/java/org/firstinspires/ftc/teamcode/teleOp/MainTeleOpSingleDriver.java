package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Pose.LimelightPose;
import org.firstinspires.ftc.teamcode.JaviVision.v3.LimelightProcessor_v3;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.ShooterTele;

@TeleOp(name="Single Driver Tele", group="Iterative OpMode")
@Config
public class MainTeleOpSingleDriver extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DrivetrainTele dt;
    private IntakeTele intake;
    ShooterTele shooter;
    private Telemetry dash;
    public static double power;
    public static double targetVelo = 1300;
    public double botHeading;
    public double cameraLoopCounter = 0;
    double lastTX;

    public GoBildaPinpointDriver odo;
    LimelightProcessor_v3 ll;
    public static double alpha = 0.8;




    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dt = new DrivetrainTele(hardwareMap, gamepad1, gamepad2, telemetry);
        intake = new IntakeTele(hardwareMap, gamepad1, gamepad2, telemetry);
        shooter = new ShooterTele(hardwareMap, gamepad1, gamepad2, telemetry, runtime);
        telemetry.addData("Status", "Initialized");
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        ll = new LimelightProcessor_v3(hardwareMap, odo);


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
        if (cameraLoopCounter == 20){
            ll.update();
            cameraLoopCounter = 0;
        }
        double raw_tx = ll.pose.x;

        double tx = alpha*lastTX + (1-alpha)*raw_tx;
        lastTX = tx;

        dt.feildCentricDrive();
        dt.updateOdo();


        //intake
        intake.updateSingle();

        //shooter
        shooter.shooterMachineSingle();

        //update power var
        power = shooter.power;

        //telemetry
        /*
        telemetry.addData("llx", ll.pose.x*39.3701);
        telemetry.addData("tx", tx);

        telemetry.addData("llpx", ll.pose.posX*39.3701);
        telemetry.addData("loop counter", cameraLoopCounter);

         */

        telemetry.addData("rot", dt.botHeadingPIN());

        dash.addData("shooter vel", shooter.getMotorVel());
        dash.addData("target vel", targetVelo);
        dash.addData("flywheel rpm", shooter.getMotorRPM());
        telemetry.update();
        dash.update();
        cameraLoopCounter += 1;
    }

    @Override
    public void stop() {
    }
}
