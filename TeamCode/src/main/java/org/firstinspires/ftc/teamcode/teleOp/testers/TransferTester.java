package org.firstinspires.ftc.teamcode.teleOp.testers;

import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JaviVision.Position.FinalPositionV3.LimelightProcessor_v3Tele;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.RobotActions;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Lights;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;


@TeleOp(name="Transfer Tester", group="Iterative OpMode")
@Config
public class TransferTester extends OpMode {

    //choose
    private OLDChoose choose;

    //runtime
    private ElapsedTime overallRuntime;
    private double lastTime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private double turretAngle = 0.48;

    //localization
    private Follower follower;

    //robot
    private RobotActions robot;
    private OLDChoose.Alliance currentColor = RED;

    LimelightProcessor_v3Tele ll;
    private Telemetry dash;
    public static double transferV;
    public static double tuner;



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

        //telemetry
        telemetry.addData("Status", "Initialized");

        //robot
        robot = new RobotActions(gamepad1, gamepad2, drivetrain, intake, shooter, follower, overallRuntime, telemetry, new Lights(hardwareMap, new ElapsedTime(), telemetry));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
        currentColor = RED;
        choose.allianceInit();
        currentColor = choose.getSelectedAlliance();
        telemetry.update();
    }

    @Override
    public void start() {
        overallRuntime.reset();
        if(currentColor == RED){
            follower.setPose(new Pose(115, 70, Math.PI/2));
        }
        if(currentColor == BLUE){
            follower.setPose(new Pose(29, 70, Math.PI/2));
        }
    }

    @Override
    public void loop() {
        intake.setTransferVelPID(transferV, intake.getTransferVel(), 0, tuner);
        //2250

        dash.addData("Current vel", intake.getTransferVel());
        dash.addData("Target vel", transferV);
        dash.update();

    }

    @Override
    public void stop() {
    }
}
