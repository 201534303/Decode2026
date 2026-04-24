// codex was here lmao
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDChoose;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Feedback;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Intake;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

public class RobotActions {
    private static final double GRAVITY_IN_PER_S2 = -386.09;
    private static final double BASE_LAUNCH_ANGLE_DEG = 40.80591944;
    private static final double HOOD_ANGLE_SCALE_DEG = 21.54833298;
    private static final double BASE_LAUNCH_HEIGHT_IN = 11.56594488;
    private static final double LAUNCH_HEIGHT_SIN_SCALE_IN = 4.3897638;
    private static final double TARGET_HEIGHT_SLOPE = -0.0571429;
    private static final double TARGET_HEIGHT_INTERCEPT_IN = 51.14286;
    private static final int MAX_LEAD_ITERATIONS = 4;
    private static final double LEAD_TIME_TOLERANCE_S = 1e-3;
    private static final double UNLOCK_ASSIST_MS = 200.0;
    private static final double FAR_ZONE_DISTANCE_IN = 130.0;
    private static final double MAX_SHOT_LEAD_SPEED_FAR_IN_PER_S = 8.0;
    private static final double MAX_SHOT_LEAD_SPEED_CLOSE_IN_PER_S = 16.0;

    private static final double FLANKPOSOTION = 12;
    private double shooterVelocityOffset = 0.0;
    private double hoodOffset = 0.0;
    private double flankPositionOffset = 0.0;
    private double lastBaseShooterVelocity = 0.0;
    private double lastAppliedShooterVelocity = 0.0;
    private double lastBaseHood = 0.0;
    private double lastAppliedHood = 0.0;
    private double lastAppliedFlankPosition = FLANKPOSOTION;

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
    private boolean wheelsLocked = false;
    private double unlockAssistUntilMs = Double.NEGATIVE_INFINITY;
    //telemetry`
    Telemetry telemetry;

    //runtime
    private ElapsedTime overallRuntime;

    //subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Feedback feedback;

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
    private boolean singleDriver = false;
    private boolean liftMode = false;
    public RobotActions (Gamepad g1, Gamepad g2, Drivetrain dt, Intake in, Shooter sh, Follower fo, ElapsedTime ru, Telemetry te, Feedback fe){
        gamepad1 = g1;
        gamepad2 = g2;
        drivetrain = dt;
        intake = in;
        shooter = sh;
        follower = fo;
        overallRuntime = ru;
        telemetry = te;
        feedback = fe;
    }

    public RobotActions (Shooter sh, Follower fo, Telemetry te){
        shooter = sh;
        follower = fo;
        telemetry = te;
    }

    public void setLocalizationBack() {
        setLocalizationPose(HOMING);
    }
    public void setLocalizationOurSide(OLDChoose.Alliance currentColor) {
        if (currentColor == OLDChoose.Alliance.RED){
            setLocalizationPose(HOMINGRED);
        }
        else{
            setLocalizationPose(HOMINGBLUE);
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

    public void fieldCentricDrive(OLDChoose.Alliance currentColor, double botHeadingaForMatrix, Vector vel){
        double yMove = -gamepad1.right_stick_y; //Y stick value is reversed
        double xMove = gamepad1.right_stick_x;
        double rot = 0;
        double robotSpeed = vel != null ? vel.getMagnitude() : 0.0;

        boolean doIt = (singleDriver && gamepad1.dpad_right) || (!singleDriver && gamepad1.left_bumper);
        if(doIt){
            double idealRot = Math.PI/2;
            if(currentColor == OLDChoose.Alliance.RED){
                idealRot -= OFFSETROTATIONGATE;
            }
            else{
                idealRot += OFFSETROTATIONGATE;
            }
            double error = angleDiffRad(botHeadingaForMatrix, idealRot);
            rot = rotationPID.calculate(error);
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

        if (gamepad1.rightBumperWasPressed()){
            wheelsLocked = true;
            unlockAssistUntilMs = Double.NEGATIVE_INFINITY;
        }
        else if (gamepad1.rightBumperWasReleased()){
            wheelsLocked = false;
            if (overallRuntime != null) {
                unlockAssistUntilMs = overallRuntime.milliseconds() + UNLOCK_ASSIST_MS;
            }
        }
        
        if(wheelsLocked){
            drivetrain.lock();
            drivetrain.setMotorPowers(0,0,0,0);
        }
        else{
            drivetrain.unlock();
            if (isUnlockAssistActive()) {
                drivetrain.setMotorPowers(1,-1,1,-1);
            } else if (brake > 0.9){
                drivetrain.setMotorPowers(frontLeftPower * 0.6, backLeftPower * 0.6, frontRightPower * 0.6, backRightPower * 0.6);
            } else if (superBrake > 0.9) {
                drivetrain.setMotorPowers(frontLeftPower * 0.25, backLeftPower * 0.25, frontRightPower * 0.25, backRightPower * 0.25);
            } else{
                drivetrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            }
        }

        if (feedback != null) {
            feedback.updateWheelLockState(wheelsLocked);
        }
    }

    private boolean isUnlockAssistActive() {
        return overallRuntime != null && overallRuntime.milliseconds() < unlockAssistUntilMs;
    }

    public static double angleDiffRad(double fromRad, double toRad) {
        double diff = (toRad - fromRad) % (2.0 * Math.PI);
        if (diff > Math.PI) diff -= 2.0 * Math.PI;
        if (diff < -Math.PI) diff += 2.0 * Math.PI;
        return diff; // range: [-PI, PI]
    }

    private void setLocalizationPose(Pose targetPose) {
        boolean changed = !isSamePose(follower.getPose(), targetPose);
        follower.setPose(targetPose);
        if (changed && feedback != null) {
            feedback.notifyRelocalized();
        }
    }

    private boolean isSamePose(Pose a, Pose b) {
        return Math.abs(a.getX() - b.getX()) < 0.01
                && Math.abs(a.getY() - b.getY()) < 0.01
                && Math.abs(angleDiffRad(a.getHeading(), b.getHeading())) < Math.toRadians(0.5);
    }


    public void updateIntake(){
        if(singleDriver){
            if(gamepad1.right_bumper){
                intake.setIntPower(1);
            }
            else{
                intake.setIntPower(.1);
            }
        }
        else{
            if(gamepad2.right_stick_y > 0 || gamepad2.right_trigger > 0.8){
                intake.setIntPower(gamepad2.right_stick_y + 0.1);
            }
            else{
                intake.setIntPower(0.1);
            }
        }

        intake.intakeIn();
        intake.intakeMachine();
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

        if(singleDriver){
            if(gamepad1.left_bumper){
                intake.setTransferPower(1);

            }
            else{
                intake.setTransferPower(0);
            }
        }
        else{
            if(Math.abs(gamepad2.left_stick_y) > 0.05 && ((-gamepad2.left_stick_y > 0 && Math.abs(turAngle) < 72 && dist >= 55 && Math.abs(gamepad1.left_stick_x) < 0.15) || gamepad2.right_trigger > 0.8)){
                intake.setTransferPower(-gamepad2.left_stick_y);
            }
            else{
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
        double dist = distanceToTarget(currentColor, x, y);
        double usedVelX = vel != null ? vel.getXComponent() : 0.0;
        double usedVelY = vel != null ? vel.getYComponent() : 0.0;
        double usedRobotSpeed = vel != null ? vel.getMagnitude() : 0.0;
        double maxLeadSpeed = dist > FAR_ZONE_DISTANCE_IN
                ? MAX_SHOT_LEAD_SPEED_FAR_IN_PER_S
                : MAX_SHOT_LEAD_SPEED_CLOSE_IN_PER_S;

        // Cap the moving-shot lead so fast drive corrections do not over-predict.
        if (usedRobotSpeed > maxLeadSpeed && usedRobotSpeed > 1e-6) {
            double scale = maxLeadSpeed / usedRobotSpeed;
            usedVelX *= scale;
            usedVelY *= scale;
            usedRobotSpeed = maxLeadSpeed;
        }

        double time = calculateIterativeLeadTime(currentColor, x, y, usedVelX, usedVelY);

        double virtualX = x + time * usedVelX;
        double virtualY = y + time * usedVelY;
        telemetry.addData("virtualX", virtualX);
        telemetry.addData("virtualY", virtualY);
        telemetry.addData("virtualXchange", time * usedVelX);
        telemetry.addData("virtualYchange", time * usedVelY);
        telemetry.addData("leadVelocityMagnitude", usedRobotSpeed);

        updateTransfer(currentColor, vel, virtualX, virtualY, false);
        updateTurret(currentColor, virtualX, virtualY, heading);
        updateShooter(currentColor, virtualX, virtualY, usedRobotSpeed);
    }

    public void updateConversion(OLDChoose.Alliance currentColor, boolean turretOn, double x, double y, double heading, Vector vel, double rVel, double mul) {
        double time = calculateIterativeLeadTime(currentColor, x, y, vel) * 2.0;

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
        return time(OLDChoose.Alliance.RED, x, y);
    }

    public double time(OLDChoose.Alliance currentColor, double x, double y){
        double dist = distanceToTarget(currentColor, x, y);
        double flightTime = noDragBallTime(dist);
        if (!Double.isFinite(flightTime) || flightTime < 0.0) {
            return 0.0;
        }
        return flightTime;
    }

    // Recompute flight time from the predicted future position a few times so
    // the moving-shot lead converges instead of relying on a single estimate.
    public double calculateIterativeLeadTime(OLDChoose.Alliance currentColor, double x, double y, Vector vel) {
        double velX = vel != null ? vel.getXComponent() : 0.0;
        double velY = vel != null ? vel.getYComponent() : 0.0;
        return calculateIterativeLeadTime(currentColor, x, y, velX, velY);
    }

    private double calculateIterativeLeadTime(OLDChoose.Alliance currentColor, double x, double y, double velX, double velY) {
        double flightTime = time(currentColor, x, y);
        if (flightTime <= 0.0) {
            return 0.0;
        }

        for (int i = 0; i < MAX_LEAD_ITERATIONS; i++) {
            double virtualX = x + flightTime * velX;
            double virtualY = y + flightTime * velY;
            double nextFlightTime = time(currentColor, virtualX, virtualY);

            if (nextFlightTime <= 0.0) {
                return flightTime;
            }
            if (Math.abs(nextFlightTime - flightTime) < LEAD_TIME_TOLERANCE_S) {
                return nextFlightTime;
            }

            flightTime = nextFlightTime;
        }

        return flightTime;
    }

    private double noDragBallTime(double distance) {
        if (!Double.isFinite(distance)) {
            return Double.NaN;
        }

        double hood = lookupHood(distance);
        double thetaDeg = BASE_LAUNCH_ANGLE_DEG + hood * HOOD_ANGLE_SCALE_DEG;
        double thetaRad = Math.toRadians(thetaDeg);
        double cosTheta = Math.cos(thetaRad);
        if (Math.abs(cosTheta) < 1e-9) {
            return Double.NaN;
        }

        double tanTheta = Math.tan(thetaRad);
        double launchHeightIn = launchHeightForTheta(thetaDeg);
        double targetHeightIn = targetHeightForDistance(distance);
        double verticalDisplacementIn = targetHeightIn - launchHeightIn;

        double tSquared = (2.0 * (verticalDisplacementIn - distance * tanTheta)) / GRAVITY_IN_PER_S2;
        if (tSquared <= 0.0) {
            return Double.NaN;
        }

        return Math.sqrt(tSquared);
    }

    private double lookupHood(double distance) {
        if (distance > 133.0) {
            return 0.00000752141*distance*distance*distance-0.00358577*distance*distance+0.568796*distance-29.52802;
        }
        if (distance > 55) {
            return -0.00470013*distance+1.25855;
        }
        return 1.0;
    }

    private double launchHeightForTheta(double thetaDeg) {
        return BASE_LAUNCH_HEIGHT_IN
                + LAUNCH_HEIGHT_SIN_SCALE_IN * Math.sin(Math.toRadians(90.0 - thetaDeg));
    }

    private double targetHeightForDistance(double distance) {
        return TARGET_HEIGHT_SLOPE * distance + TARGET_HEIGHT_INTERCEPT_IN;
    }

    private double distanceToTarget(OLDChoose.Alliance currentColor, double posX, double posY) {
        if(currentColor == OLDChoose.Alliance.BLUE){
            return Math.hypot(-posX, 144 - posY);
        }
        return Math.hypot(144 - posX, 144 - posY);
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
    public void toggleSingleDriver(){
        singleDriver = !singleDriver;
    }
    public void toggleLiftMode(){
        liftMode = !liftMode;
    }

    public void adjustShooterVelocityOffset(double delta) {
        shooterVelocityOffset += delta;
    }

    public void adjustHoodOffset(double delta) {
        hoodOffset += delta;
    }

    public void adjustFlankPositionOffset(double delta) {
        flankPositionOffset += delta;
    }

    public double getShooterVelocityOffset() {
        return shooterVelocityOffset;
    }

    public double getHoodOffset() {
        return hoodOffset;
    }

    public double getFlankPositionOffset() {
        return flankPositionOffset;
    }

    public double getBaseFlankPosition() {
        return FLANKPOSOTION;
    }

    public double getLastBaseShooterVelocity() {
        return lastBaseShooterVelocity;
    }

    public double getLastAppliedShooterVelocity() {
        return lastAppliedShooterVelocity;
    }

    public double getLastBaseHood() {
        return lastBaseHood;
    }

    public double getLastAppliedHood() {
        return lastAppliedHood;
    }

    public double getLastAppliedFlankPosition() {
        return lastAppliedFlankPosition;
    }

    public void updateTurret(OLDChoose.Alliance currentColor, double posX, double posY, double h){
        this.posX = posX;
        this.posY = posY;

        double heading = Math.toDegrees(h);
        double flankPosition = FLANKPOSOTION + flankPositionOffset;
        lastAppliedFlankPosition = flankPosition;

        double turretAngle = 0;

        if(currentColor == OLDChoose.Alliance.BLUE){
            //targets (0, 124), (20, 144)
            double delX1 = 0 - posX;
            double delY1 = 144 - flankPosition - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = flankPosition - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = averageAngle(turretAngle1, turretAngle2);
        }

        if(currentColor == OLDChoose.Alliance.RED){
            //targets (144, 124), (124, 144)
            double delX1 = 144 - posX;
            double delY1 = 144 - flankPosition - posY;
            double turretAngle1 = Math.toDegrees(Math.atan2(delY1, delX1)) - (heading);
            double delX2 = 144 - flankPosition - posX;
            double delY2 = 144 - posY;
            double turretAngle2 = Math.toDegrees(Math.atan2(delY2, delX2)) - (heading);
            turretAngle = averageAngle(turretAngle1, turretAngle2);
        }


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
        double dist = distanceToTarget(currentColor, posX, posY);


        double speed = 0;
        double hood = lookupHood(dist);


        if(dist > 133){//far zone
            speed = 0.0535558*dist*dist-12.40579*dist+2093.01359;
            //1186.66887
            //3.63909
            //1048.1478
        }
        else if(dist > 55){
            speed = 0.0813636*dist*dist-11.36087*dist+1487.29396;
        }
        else{
            speed = 1108.571;
        }

        if(speed < 0){
            speed = 0;
        }

        if(liftMode){
            hood = 1.0;
        }

        lastBaseShooterVelocity = speed;
        lastBaseHood = hood;

        speed = Math.max(0, speed + shooterVelocityOffset);
        hood = clamp(hood + hoodOffset, 0.0, 1.0);

        lastAppliedShooterVelocity = speed;
        lastAppliedHood = hood;

        shooter.setHood(hood);
        shooter.flywheelSpinDynamic(speed, shooter.getMotorVel(), robotVel);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
