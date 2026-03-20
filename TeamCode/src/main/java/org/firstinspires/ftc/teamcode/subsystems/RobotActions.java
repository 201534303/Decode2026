package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
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

    //DELETE LATER
    public double DELETEBUTTHISISVEL = 1720;
    public double DELETEBUTTHISISHOOD = 0.3;
    public double DELETEBUTTHISISTURRET = 0.48;
    private final double[] noiseReduction = new double[3]; // last 3 turret angles
    private int noiseReductionCount = 0; // for startup (avoid averaging in zeros)
    //gamepads
    Gamepad gamepad1, gamepad2;

    private PIDController rotationPID = new PIDController(0.9, 0, 0);
    private final double OFFSETROTATIONGATE = Math.toRadians(56);
    //telemetry`
    Telemetry telemetry;

    //runtime
    private ElapsedTime overallRuntime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Lights light;

    //localization
    private Follower follower;

    //constant poses
    private final Pose HOMING = new Pose(72, 3, Math.toRadians(90));
    private final Pose HOMINGRED = new Pose(7.7, 3, Math.toRadians(90));
    private final Pose HOMINGBLUE = new Pose(144-7.7, 3, Math.toRadians(90));

    //variables
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
    private double speedDif;
    private boolean noahMode = true;
    private boolean liftMode = false;
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

    public RobotActions (Shooter sh, Follower fo, Telemetry te){
        shooter = sh;
        follower = fo;
        telemetry = te;
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

    public void setIMUZero(double x, double y, OLDChoose.Alliance currentColor) {
        if(currentColor == OLDChoose.Alliance.RED){
            follower.setPose(new Pose(x, y, 0));
        }
        else{
            follower.setPose(new Pose(x, y, Math.PI));
        }
    }

    public void fieldCentricDrive(OLDChoose.Alliance currentColor, double botHeadingaForMatrix){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = 0;
        if(gamepad1.left_bumper){
            double idealRot = Math.PI/2;
            if(currentColor == OLDChoose.Alliance.RED){
                idealRot -= OFFSETROTATIONGATE;
            }
            else{
                idealRot += OFFSETROTATIONGATE;
            }
            double error = angleDiffRad(botHeadingaForMatrix, idealRot);
            telemetry.addData("idealRot", idealRot);
            telemetry.addData("error", error);
            rot = rotationPID.calculate(error);
            telemetry.addData("rot", rot);
        }
        else{
            rot = gamepad1.left_stick_x;
        }

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
    public static double angleDiffRad(double fromRad, double toRad) {
        double diff = (toRad - fromRad) % (2.0 * Math.PI);
        if (diff > Math.PI) diff -= 2.0 * Math.PI;
        if (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff; // range: [-PI, PI]
    }


    public void updateIntake(){
        if(noahMode){
            if(gamepad2.right_stick_y > 0 || gamepad2.right_trigger > 0.8){
                intake.setIntPower(gamepad2.right_stick_y + 0.1);
            }
            else{
                intake.setIntPower(0.1);
            }
        }

        else{
            intake.setIntPower(-gamepad2.right_stick_y + 0.1);
        }

        intake.intakeIn();
        intake.intakeMachine();
        if (intake.haveBall()){
            gamepad2.rumble(500);
            //gamepad1.rumble(500);
        }
    }

    public void updateTransfer(OLDChoose.Alliance currentColor, Vector vel, double posX, double posY, boolean rotating) {

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
        double speedMul = 5.0;

        //if(dist > 140){
          //  speedMul = 3.0;
       // }

        telemetry.addData("moving mag", vel.getMagnitude());
        telemetry.addData("shooting dif", speedDif);

        if(noahMode){
            if((-gamepad2.left_stick_y > 0 || gamepad2.right_trigger > 0.8) && Math.abs(gamepad2.left_stick_y) > 0.05 && Math.abs(turAngle) < 72 && dist >= 55){
                intake.setTransferVelPID(-gamepad2.left_stick_y * speedMul * 2250, intake.getTransferVel(), 0, 0);
            }
            else{
                //intake.setTransferVelPID(0, intake.getTransferVel(),0,0);
                intake.setTransferPower(0);
            }
        }
        else{
            if(!rotating && Math.abs(gamepad2.left_stick_y) > 0.05 && vel.getMagnitude() < 20 && Math.abs(turAngle) < 72 && dist >= 66){
                intake.setTransferVelPID(-gamepad2.left_stick_y * speedMul * 2250, intake.getTransferVel(), 0, 0);
            }
            else{
                //intake.setTransferVelPID(0, intake.getTransferVel(),0,0);
                intake.setTransferPower(0);
            }
        }
    }
    public void updateTransfer() {

        if(Math.abs(gamepad2.left_stick_y) > 0.05){
            intake.setTransferVelPID(-gamepad2.left_stick_y * 0.6 * 2250, intake.getTransferVel(), 0, 0);
        }
        else{
            //intake.setTransferVelPID(0, intake.getTransferVel(),0,0);
            intake.setTransferPower(0.1);
        }
    }

    //UPDATE

    public void update(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel, double rVel) {
        double time = time(x,y);

        double virtualX = x + time*vel.getXComponent();
        double virtualY = y + time*vel.getYComponent();
        telemetry.addData("virtualX", virtualX);
        telemetry.addData("virtualY", virtualY);
        telemetry.addData("virtualXchange", time*vel.getXComponent());
        telemetry.addData("virtualYchange", time*vel.getYComponent());

        updateTurret(currentColor, virtualX, virtualY, heading);
        updateShooter(currentColor, virtualX, virtualY, vel.getMagnitude());
    }

    public void updateConversion(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel, double rVel, double mul) {
        double time = time(x, y) * 2.0;

        double virtualX = x + time * vel.getXComponent();
        double virtualY = y + time * vel.getYComponent();
        telemetry.addData("virtualX", virtualX);
        telemetry.addData("virtualY", virtualY);
        telemetry.addData("virtualXchange", time * vel.getXComponent());
        telemetry.addData("virtualYchange", time * vel.getYComponent());

        if (turretOn) {
            updateTurretConversion(currentColor, virtualX, virtualY, heading, mul);
        }
        if (!turretOn) {
            shooter.rotateTurret(0);
        }

        //updateShooter(currentColor, virtualX, virtualY);
    }

    public double time(double x, double y){
        double dist = Math.hypot(144-x, 144-y);
        if(dist < 135){
            return 0;
        }
        double time = 0.00655284*dist-0.259554;
        return time;
    }

    public void updateShooterTesting(boolean shooterOff) {
        telemetry.addData("setHood", DELETEBUTTHISISHOOD);
        telemetry.addData("setTurret", DELETEBUTTHISISTURRET);
        telemetry.addData("setShooterVel", DELETEBUTTHISISVEL);
        shooter.setHood(DELETEBUTTHISISHOOD);
        shooter.rotateTurretZeroTest(DELETEBUTTHISISTURRET);
        if (shooterOff){
            shooter.flywheelSpinBangBang(0, shooter.getMotorVel(), 0);
        }
        else{
            shooter.flywheelSpinBangBang(DELETEBUTTHISISVEL, shooter.getMotorVel(), 0);
        }
    }
    public void toggleNoahMode(){
        //noahMode = !noahMode;
    }
    public void toggleLiftMode(){
        liftMode = !liftMode;
    }

    public void updateTurret(OLDChoose.Alliance currentColor, double posX, double posY, double h){
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
            turretAngle = averageAngle(turretAngle1, turretAngle2);
            rawX = posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        if(currentColor == OLDChoose.Alliance.RED){
            //targets (144, 124), (124, 144)
            double delX1 = 144 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 124 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = averageAngle(turretAngle1, turretAngle2);
            rawX = fieldLength - posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        telemetry.addData("turretAngle", turretAngle);

        double filteredTurretAngle = rollingAverage4(turretAngle);
        shooter.rotateTurret(filteredTurretAngle);
        turAngle = filteredTurretAngle;
    }

    private double rollingAverage4(double current) {
        double sum = current;
        int n = 1;

        int valid = Math.min(noiseReductionCount, noiseReduction.length);
        for (int i = 0; i < valid; i++) {
            sum += noiseReduction[i];
            n++;
        }

        double avg = sum / n;

        // shift right, drop oldest
        for (int i = noiseReduction.length - 1; i > 0; i--) {
            noiseReduction[i] = noiseReduction[i - 1];
        }
        noiseReduction[0] = current;
        if (noiseReductionCount < noiseReduction.length) noiseReductionCount++;

        return avg;
    }


    private void updateTurretConversion(OLDChoose.Alliance currentColor, double posX, double posY, double h, double mul){
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
            turretAngle = averageAngle(turretAngle1, turretAngle2);
            rawX = posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        if(currentColor == OLDChoose.Alliance.RED){
            //targets (144, 124), (124, 144)
            double delX1 = 144 - posX;
            double delY1 = 124 - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 124 - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = averageAngle(turretAngle1, turretAngle2);
            rawX = fieldLength - posX - CONSTX;
            rawY = fieldLength - posY - CONSTY;
            idealAngle = Math.atan(rawY/rawX);
            delAngle = Math.toDegrees(Math.atan(delY1/delX1) - idealAngle);
        }

        telemetry.addData("turretAngle", turretAngle);
        shooter.rotateTurretConversionTest(turretAngle, mul);
        turAngle = turretAngle;
    }

    private double averageAngle(double angleA, double angleB){
        double aRad = Math.toRadians(angleA);
        double bRad = Math.toRadians(angleB);

        double x = Math.cos(aRad) + Math.cos(bRad);
        double y = Math.sin(aRad) + Math.sin(bRad);

        double avgRad = Math.atan2(y, x);
        double avgDeg = Math.toDegrees(avgRad);

        // normalize to [-180, 180)
        return ((avgDeg + 180) % 360 + 360) % 360 - 180;
    }


    public void updateShooter(OLDChoose.Alliance currentColor, double posX, double posY, double robotVel) {
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
        double hood = 0;


        if(dist > 120){//far zone
            hood = 0.0;
            speed = 3.63909*dist+1114.64786;
        }
        else if(dist > 98){
            hood = 0.25;
            speed = 1096.99182 + 3.05*dist;
            // 2.7835
        }
        else if(dist > 82){
            hood = 0.000126391*dist*dist-0.0317782*dist+2.21627;
            speed = 5*dist+860;
            // 2.7835
        }
        else if(dist > 78){
            hood = .475;
            speed = 5*dist+860;
            // 2.7835
        }
        else if(dist > 55){ // close5.84356\cdot0.968317^{x}
            hood = 5.84356*Math.pow(0.968317, dist);
            speed = 5*dist+860;
        }
        else{
            hood = 1;
            speed = 1135;
        }
        if(speed < 0){
            speed = 0;
        }


        if(liftMode){
            hood = 1.0;
        }


        shooter.setHood(hood);
        shooter.flywheelSpinDynamic(speed, shooter.getMotorVel(), robotVel);
        speedDif = speed - shooter.getMotorVel();
    }
}
