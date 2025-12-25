package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.Choose;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.teleOp.MainTeleOpBetter;

public class RobotActions {

    //DELETELATER
    double DELETEBUTTHISISVEL = 900;
    double DELETEBUTTHISISHOOD = 0.2;

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
    private final Pose HOMING = new Pose(72, 3, Math.toRadians(90));

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

    public void setIMUZero(Choose.Alliance currentColor) {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        if(currentColor == Choose.Alliance.RED){
            follower.setPose(new Pose(x, y, Math.PI));
        }
        if(currentColor == Choose.Alliance.BLUE){
            follower.setPose(new Pose(x, y, 0));
        }
    }

    public void fieldCentricDrive(Choose.Alliance currentColor){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = gamepad1.left_stick_x;

        double brake = gamepad1.right_trigger;
        double superBrake = gamepad1.left_trigger;

        double botHeadingaForMatrix = follower.getPose().getHeading();

        // Rotate the movement direction counter to the bot's rotation

        if (currentColor == Choose.Alliance.RED) {
            // Flip the field coordinate system 180 degrees
            botHeadingaForMatrix += Math.PI;
        }
        botHeadingaForMatrix = - botHeadingaForMatrix;

        double rotedX = xMove * Math.cos(botHeadingaForMatrix) - yMove * Math.sin(botHeadingaForMatrix);
        double rotedY = xMove * Math.sin(botHeadingaForMatrix) + yMove * Math.cos(botHeadingaForMatrix);

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
        intake.setIntPower(-gamepad2.right_stick_y + 0.1);
        intake.intakeIn();
        intake.intakeMachine();
    }

    public void updateTransfer() {
        intake.setTransferPower(-gamepad2.left_stick_y);
    }

    public void setShootingAuto() {
        shootingMode = 0;
    }

    public void setShootingOff() {
        shootingMode = 1;
    }

    private void updateTurret(Choose.Alliance currentColor){
        double heading = Math.toDegrees(follower.getPose().getHeading());
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Theta", follower.getPose().getHeading());

        telemetry.addData("color", currentColor);

        double turretAngle = 0;

        if(currentColor == Choose.Alliance.BLUE){
            double delX = -posX;
            double delY = 144-posY;
            turretAngle = Math.toDegrees(Math.atan2(delY, delX)) - (heading);
        }

        if(currentColor == Choose.Alliance.RED){
            double delX = 144-posX;
            double delY = 144-posY;
            turretAngle = Math.toDegrees(Math.atan2(delY, delX)) - (heading);
        }

        shooter.rotateTurret(turretAngle);
    }

    public void update(Choose.Alliance currentColor) {
        updateTurret(currentColor);
        updateShooter(currentColor);
        shooter.flywheelSpin(DELETEBUTTHISISVEL, shooter.getMotorVel(), 0);
        shooter.setHood(DELETEBUTTHISISHOOD);
        telemetry.addData("shooterVel", DELETEBUTTHISISVEL);
        telemetry.addData("shooterHood", DELETEBUTTHISISHOOD);
    }

    public void shooter(double velChange, double hooodChange){
        DELETEBUTTHISISVEL += velChange;
        DELETEBUTTHISISHOOD += hooodChange;
        telemetry.addData("We are in shooter function", "Thats right Javi!");
    }

    private void updateShooter(Choose.Alliance currentColor) {
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        double dist;

        if(currentColor == Choose.Alliance.BLUE){
            double delX = -posX;
            double delY = 144-posY;
            dist = Math.hypot(delX, delY);
        }
        if(currentColor == Choose.Alliance.RED){
            double delX = 144-posX;
            double delY = 144-posY;
            dist = Math.hypot(delX, delY);
        }

        /*if(shootingMode == 0){
            if((posY > 120-posX && posY > -24+posX )|| (posY < 120-posX && posY < -24+posX)){

                shooter.setHood(1-);
                shooter.setVel();
            }
        }

         */

    }

}
