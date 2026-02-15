package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Lights;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;

public class RobotActions {

    //DELETELATER
    public double DELETEBUTTHISISVEL = 1720;
    public double DELETEBUTTHISISHOOD = 0.3;
    public double DELETEBUTTHISISTURRET = 0.48;

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
    private Lights light;

    //constants
    private final Pose HOMING = new Pose(72, 3, Math.toRadians(90));
    private final Pose HOMINGRED = new Pose(7.7, 3, Math.toRadians(90));
    private final Pose HOMINGBLUE = new Pose(144-7.7, 3, Math.toRadians(90));

    //variables
    private int shootingMode = 1;
    public double turAngle;
    public double delAngle;
    public double idealAngle;
    public double rawX;
    public double rawY;
    public double posX;
    public double posY;
    private final double CONSTX = 16;
    private final double CONSTY = 13.375;
    private final double fieldLength = 144;

    public RobotActions (Gamepad g1, Gamepad g2, Drivetrain dt, Intake in, Shooter sh, Follower fo, ElapsedTime ru, Telemetry te, Lights li){
        gamepad1 = g1;
        gamepad2 = g2;
        drivetrain = dt;
        intake = in;
        shooter = sh;
        follower = fo;
        overallRuntime = ru;
        telemetry = te;
        light = li;
    }

    public void setLocalizationBack() {
        follower.setPose(HOMING);
    }
    public void setLocalizationOurSide(OLDChoose.Alliance currentColor) {
        if (currentColor == OLDChoose.Alliance.RED){
            follower.setPose(HOMINGRED);
        }
        else{
            follower.setPose(HOMINGBLUE);
        }
    }


    public void setIMUZero(OLDChoose.Alliance currentColor, double x, double y) {
        if(currentColor == OLDChoose.Alliance.RED){
            follower.setPose(new Pose(x, y, 0));
        }
        if(currentColor == OLDChoose.Alliance.BLUE){
            follower.setPose(new Pose(x, y, Math.PI));
        }
    }

    public void fieldCentricDrive(OLDChoose.Alliance currentColor, double botHeadingaForMatrix){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = gamepad1.left_stick_x;

        double brake = gamepad1.right_trigger;
        double superBrake = gamepad1.left_trigger;

        // Rotate the movement direction counter to the bot's rotation

        if (currentColor == OLDChoose.Alliance.BLUE) {
            // Flip the field coordinate system 180 degrees
            botHeadingaForMatrix += Math.PI;
        }
        botHeadingaForMatrix = - botHeadingaForMatrix;

        double rotedX = xMove * Math.cos(botHeadingaForMatrix) - yMove * Math.sin(botHeadingaForMatrix);
        double rotedY = xMove * Math.sin(botHeadingaForMatrix) + yMove * Math.cos(botHeadingaForMatrix);

        rotedX = rotedX * 1.1 ;  // Counteract imperfect strafing

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
        if (intake.haveBall()){
            gamepad2.rumble(500);
        }
    }

    public void updateTransfer(OLDChoose.Alliance currentColor, Vector vel, double posX, double posY) {
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();
        double total = Math.hypot(velY, velX);

        double delY = 0;
        double delX = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            delX = -posX;
            delY = 144-posY;
        }

        if(currentColor == OLDChoose.Alliance.RED){
            delX = 144-posX;
            delY = 144-posY;
        }

        double dist = Math.hypot(delY, delX);
        double speedMul = 0.60;

        if(dist > 140){
            speedMul = 0.56;
        }


        if(total < 3 && Math.abs(gamepad2.left_stick_y) > 0.05){
            intake.setTransferVelPID(-gamepad2.left_stick_y * speedMul * 2250, intake.getTransferVel(), 0, 0);
        }
        else{
            //intake.setTransferVelPID(0, intake.getTransferVel(),0,0);
            intake.setTransferPower(0.1);
        }
    }

    //UPDATE

    public void update(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel, double rVel) {
        double[] velocities = getVelocities(currentColor, vel, x, y);
        //double rVel = velocities[0];
        double tVel = velocities[0];
        if (turretOn){
            updateTurret(currentColor, tVel, x, y, heading);
        }
        if (!turretOn){
            shooter.rotateTurret(0);
        }
        updateShooter(currentColor, rVel, x, y, rVel);
    }
    public void updateTurretTest(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel, double rVel, double mul) {
        double[] velocities = getVelocities(currentColor, vel, x, y);
        //double rVel = velocities[0];
        double tVel = velocities[0];
        if (turretOn){
            updateTurretTest(currentColor, tVel, x, y, heading, mul);
        }
        if (!turretOn){
            shooter.rotateTurret(0);
        }
        updateShooter(currentColor, rVel, x, y, rVel);
    }


    public void updateShooterTesting(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel) {
        double[] velocities = getVelocities(currentColor, vel, x, y);
        double rVel = velocities[0];
        double tVel = velocities[0];
        //updateTurret(currentColor, tVel, x, y, heading);
        //shooter.rotateTurret(0);
        telemetry.addData("hood", DELETEBUTTHISISHOOD);
        telemetry.addData("turret", DELETEBUTTHISISTURRET);
        telemetry.addData("shooterVel", DELETEBUTTHISISVEL);
        shooter.setHood(DELETEBUTTHISISHOOD);
        shooter.rotateTurret(DELETEBUTTHISISTURRET);
        shooter.flywheelSpin(DELETEBUTTHISISVEL, shooter.getMotorVel(), 0);
    }

    private void updateTurretTest(OLDChoose.Alliance currentColor, double tVel, double posX, double posY, double h, double mul){
        this.posX = posX;
        this.posY = posY;
        double heading = Math.toDegrees(h);
        double turretAngle = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            //targets (0, 124), (20, 144)
            double delX1 = 0 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 20 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = (turretAngle1 + turretAngle2)/2.0;
            rawX = posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        if(currentColor == OLDChoose.Alliance.RED){
            //targets (144, 124), (124, 144)
            //X = 7.7
            //y = 4.5
            double delX1 = 144 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 124 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = (turretAngle1 + turretAngle2)/2.0;
            rawX = fieldLength - posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        shooter.rotateTurret(turretAngle*1.05);
        turAngle = turretAngle;
    }

    private void updateTurret(OLDChoose.Alliance currentColor, double tVel, double posX, double posY, double h){
        this.posX = posX;
        this.posY = posY;
        double heading = Math.toDegrees(h);
        double turretAngle = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            //targets (0, 124), (20, 144)
            double delX1 = 0 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 20 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = (turretAngle1 + turretAngle2)/2.0;
            rawX = posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        if(currentColor == OLDChoose.Alliance.RED){
            //targets (144, 124), (124, 144)
            //X = 7.7
            //y = 4.5
            double delX1 = 144 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 124 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = (turretAngle1 + turretAngle2)/2.0;
            rawX = fieldLength - posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        shooter.rotateTurret(turretAngle);
        turAngle = turretAngle;
    }


    private void updateShooter(OLDChoose.Alliance currentColor, double rVel, double posX, double posY, double regTest) {
        double dist = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            double delX = -posX;
            double delY = 144-posY;
            dist = Math.hypot(delX, delY);
        }
        if(currentColor == OLDChoose.Alliance.RED){
            double delX = 144-posX;
            double delY = 144-posY;
            dist = Math.hypot(delX, delY);
        }

        double speed = 0;

        if(dist > 120){//far zone
            shooter.setHood(0.225);
            speed = -1252.949 + 593.005*Math.log(dist);
            //593.005
        }
        else if(dist > 85) { //most of near zone
            shooter.setHood(0.35);
            speed = 0.723027*dist+1458.89853;
        }
        else if(dist > 76){ //getting close
            //shooter.setHood(-0.0197368*dist+1.75368);
            shooter.setHood(0.45);

            //1.75368
            speed = 0.673027*dist+1435.89853;
        }
        else if(dist > 60){ //CRAZY close
            //shooter.setHood(-0.0357143*dist+2.85714);
            shooter.setHood(1);
            speed = -0.216728*dist*dist + 36.10864*dist - 40;
        }
        else{

        }

        if(speed < 0){
            speed = 0;
        }

        shooter.flywheelSpin(speed, shooter.getMotorVel(), rVel);
    }

    private double[] getVelocities(OLDChoose.Alliance currentColor, Vector vel, double posX, double posY) {
        double velX = vel.getXComponent();
        double velY = vel.getYComponent();

        double delX = 0;
        double delY = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            delX = -posX;
            delY = 144-posY;
        }

        if(currentColor == OLDChoose.Alliance.RED){
            delX = 144-posX;
            delY = 144-posY;
        }

        double rMag = Math.hypot(delX, delY);
        double rHatX = delX / rMag;
        double rHatY = delY / rMag;

        double vRadial = velX * rHatX + velY * rHatY;

        double tHatX = rHatY;
        double tHatY = -rHatX;

        double vTangential = velX * tHatX + velY * tHatY;

        return new double[] { vRadial, vTangential };
    }
}
