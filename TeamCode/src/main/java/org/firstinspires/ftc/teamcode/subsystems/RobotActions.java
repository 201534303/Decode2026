package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
    public double DELETEBUTTHISISVEL = 1520;
    public double DELETEBUTTHISISHOOD = 0.3;

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
            follower.setPose(new Pose(x, y, 0));
        }
        if(currentColor == Choose.Alliance.BLUE){
            follower.setPose(new Pose(x, y, Math.PI));
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

        if (currentColor == Choose.Alliance.BLUE) {
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
            drivetrain.setMotorPowers(frontLeftPower * 0.25, backLeftPower * 0.25, frontRightPower * 0.25, backRightPower * 0.25);
        } else{
            drivetrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        }
    }

    public void updateIntake(){
        intake.setIntPower(-gamepad2.right_stick_y + 0.1);
        intake.intakeIn();
        intake.intakeMachine();
    }

    public void updateTransfer(Choose.Alliance currentColor) {
        Vector vel = follower.getVelocity();
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();
        double total = Math.hypot(velY, velX);

        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        double delY = 0;
        double delX = 0;

        if(currentColor == Choose.Alliance.BLUE){
            delX = -posX;
            delY = 144-posY;
        }

        if(currentColor == Choose.Alliance.RED){
            delX = 144-posX;
            delY = 144-posY;
        }

        double dist = Math.hypot(delY, delX);
        double speedMul = 1;

        if(dist > 140){
            speedMul = 0.8;
        }

        telemetry.addData("dist", dist);

        if(total < 3){
            intake.setTransferPower(-gamepad2.left_stick_y * speedMul);
        }
        else{
            intake.setTransferPower(0);
        }
    }

    public void setShootingAuto() {
        shootingMode = 0;
    }

    public void setShootingOff() {
        shootingMode = 1;
    }

    //UPDATE

    public void update(Choose.Alliance currentColor, boolean turretOn) {
        double[] velocities = getVelocities(currentColor);
        double rVel = velocities[0];
        double tVel = velocities[0];
        if (turretOn){
            updateTurret(currentColor, tVel);
        }
        if (!turretOn){
            shooter.rotateTurret(0);
        }
        updateShooter(currentColor, rVel);
    }

    private void updateTurret(Choose.Alliance currentColor, double tVel){
        double heading = Math.toDegrees(follower.getPose().getHeading());
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY() + 5;
        double[] cordinates = subsystemFieldXY(posX, posY, follower.getPose().getHeading());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        telemetry.addData("Theta", follower.getPose().getHeading());

        telemetry.addData("color", currentColor);

        double turretAngle = 0;

        if(currentColor == Choose.Alliance.BLUE){
            //target (0, 144)
            double delX = 0 - cordinates[0];
            double delY = 144 - cordinates[1];
            turretAngle = Math.toDegrees(Math.atan2(delY, delX)) - (heading);
        }

        if(currentColor == Choose.Alliance.RED){
            //target (144,144)
            double delX = 144-cordinates[0];
            double delY = 144-cordinates[1];
            turretAngle = Math.toDegrees(Math.atan2(delY, delX)) - (heading);
        }

        shooter.rotateTurret(turretAngle);
    }


    private void updateShooter(Choose.Alliance currentColor, double rVel) {
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        double dist = 0;

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

        double speed = 0;

        if(dist > 120){//far zone
            shooter.setHood(0.05);
            speed = -1265.949 + 593.055*Math.log(dist);
        }
        else if(dist > 77) { //most of near zone
            shooter.setHood(0.15);
            speed = 0.673027*dist+1458.89853;
        }
        else if(dist > 72){ //getting close
            shooter.setHood(-0.0197368*dist+1.67368);
            speed = 0.673027*dist+1458.89853;
        }
        else{ //CRAZY close
            shooter.setHood(-0.0357143*dist+2.85714);
            speed = -0.216728*dist*dist + 36.10864*dist + 25;
        }

        if(speed < 0){
            speed = 0;
        }

        telemetry.addData("speed", speed);

        shooter.flywheelSpin(speed, shooter.getMotorVel(), 0);

        telemetry.addData("distance", dist);

        /*
        if(posY < 48){
            shooter.setHood(0.2);
            shooter.flywheelSpin(1610, shooter.getMotorVel(), 0);
        }
        else{
            shooter.setHood(0);
            shooter.flywheelSpin(1290, shooter.getMotorVel(), 0);
        }

         */
    }

    private double[] getVelocities(Choose.Alliance currentColor) {
        Vector vel = follower.getVelocity();
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();

        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        double delX = 0;
        double delY = 0;

        if(currentColor == Choose.Alliance.BLUE){
            delX = -posX;
            delY = 144-posY;
        }

        if(currentColor == Choose.Alliance.RED){
            delX = 144-posX;
            delY = 144-posY;
        }

        double rMag = Math.hypot(delX, delY);
        double rHatX = delX / rMag;
        double rHatY = delY / rMag;

        double vRadial = velX * rHatX + velY * rHatY;

        telemetry.addData("radial vel", vRadial);

        double tHatX = rHatY;
        double tHatY = -rHatX;

        double vTangential = velX * tHatX + velY * tHatY;

        telemetry.addData("tangent vel", vTangential);
        return new double[] { vRadial, vTangential };
    }

    public static double[] subsystemFieldXY(double x, double y, double thetaRad) {
        // Subsystem offset in ROBOT coordinates (inches)
        double dx = -5.0;  // behind center
        double dy =  0.0;  // not left/right

        // Rotate offset into FIELD coordinates
        double dxField = dx * Math.cos(thetaRad) - dy * Math.sin(thetaRad);
        double dyField = dx * Math.sin(thetaRad) + dy * Math.cos(thetaRad);

        double subX = x + dxField;
        double subY = y + dyField;

        return new double[]{subX, subY};
    }

}
