package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.teleOp.MainTeleOpBetter;
//hey
public class RobotActions {

    //gamepads
    Gamepad gamepad1, gamepad2;

    //telemetry
    Telemetry telemetry;

    //runtime
    private ElapsedTime overallRuntime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    //localization
    private Follower follower;

    //constants
    private final Pose HOMING = new Pose(72, 3, -Math.toRadians(90));

    //variables
    private int shootingMode = 1;

    public RobotActions (Gamepad g1, Gamepad g2, Drivetrain dt, Intake in, Shooter sh, Follower fo, ElapsedTime ru, Telemetry te){
        gamepad1 = g1;
        gamepad2 = g2;
        drivetrain = dt;
        intake = in;
        shooter = sh;
        follower = fo;
        overallRuntime = ru;
        telemetry = te;
    }

    public void setLocalizationBack() {
        follower.setPose(HOMING);
    }

    public void setIMUZero() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getX();
        follower.setPose(new Pose(x, y, 0));
    }

    public void fieldCentricDrive(){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = gamepad1.left_stick_x;

        double brake = gamepad1.right_trigger;
        double superBrake = gamepad1.left_trigger;

        double heading = follower.getPose().getHeading();

        double botHeading = -Math.toRadians(heading);

        // Rotate the movement direction counter to the bot's rotation
        double rotedX = xMove * Math.cos(botHeading) - yMove * Math.sin(botHeading);
        double rotedY = xMove * Math.sin(botHeading) + yMove * Math.cos(botHeading);

        rotedX = rotedX * 1.1;  // Counteract imperfect strafing

        double frontLeftPower = (rotedY + rotedX + rot);
        double backLeftPower = (rotedY - rotedX + rot);
        double frontRightPower = (rotedY - rotedX - rot);
        double backRightPower = (rotedY + rotedX - rot);

        if (brake > 0.9){
            drivetrain.setMotorPowers(frontLeftPower * 0.6, backLeftPower * 0.6, frontRightPower * 0.6, backRightPower * 0.6);
        } else if (superBrake > 0.9) {
            drivetrain.setMotorPowers(frontLeftPower * 0.35, backLeftPower * 0.35, frontRightPower * 0.35, backRightPower * 0.35);
        } else{
            drivetrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        }
    }

    public void updateIntake() {
        intake.setIntakePower(-gamepad1.right_stick_y + 0.15);
    }

    public void updateTransfer() {
        intake.setTransferPower(-gamepad1.left_stick_y);
    }

    public void setShootingAuto() {
        shootingMode = 0;
    }

    public void setShootingOff() {
        shootingMode = 1;
    }

    public void setTurret(MainTeleOpBetter.color currentColor) {
        double heading = Math.toDegrees(follower.getPose().getHeading());
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        if(currentColor == MainTeleOpBetter.color.BLUE){
            double turretAngle = (Math.toDegrees(Math.atan((144 - posX) / (144 - (Math.abs(posY)))))) - (heading + 90);
        }

        //TODO flip
        if(currentColor == MainTeleOpBetter.color.RED){
            double turretAngle = (Math.toDegrees(Math.atan((144 - posX) / (144 - (Math.abs(posY)))))) - (heading + 90);
        }

    }
}
