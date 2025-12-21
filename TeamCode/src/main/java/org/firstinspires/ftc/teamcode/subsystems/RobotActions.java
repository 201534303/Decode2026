package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;

public class RobotActions {

    //Gamepads
    private GamepadEx gamepad1, gamepad2;

    //runtime
    private ElapsedTime overallRuntime;

    //subsystems
    private Drivetrain dt;
    private Intake intake;
    private Shooter shooter;

    //localazation
    private Follower follower;

    public static double power;
    public static double targetVelo = 1300;

    public final Pose homing = new Pose(72, 3, -Math.toRadians(90));
    public Pose reset = new Pose(0, 0, 0);


}
