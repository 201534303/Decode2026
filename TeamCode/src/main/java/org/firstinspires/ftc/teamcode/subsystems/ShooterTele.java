package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterTele.shooterState.CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterTele.shooterState.FAR;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterTele.shooterState.OFF;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.supperClasses.Shooter;

public class ShooterTele extends Shooter {
    private Gamepad gamepad1, gamepad2;
    double turret = 0;
    private int mode = 0;


    private double speed = 0;
    ShooterTele.shooterState shooterState = OFF;

    public ShooterTele(HardwareMap h, Gamepad g1, Gamepad g2, Telemetry t, ElapsedTime r) {
        super(h, t, r);
        gamepad1 = g1;
        gamepad2 = g2;
    }
    public void setVel(double flywheelV){
        super.setVel(flywheelV);
    }
/*
    public void update() {
        updateRoot();
        telemetry.addData("We are here", "yeah");
        rotSpeed();
    }

 */

    public void runFlywheel(double currentV, double targetV, double kf){
        flywheelSpin(targetV, currentV, kf);
        if (gamepad2.b) {
            flywheelSpin(0, currentV, kf);
        }

    }

    public void setTurretAngle(double angle){
        //rotateTurret(0.5+angle/120);
        rotateTurret(angle);
        turret = angle;
    }





    public void setMode(int m){
        mode = m;
    }

    public int getMod(){
        return mode;
    }

    public double getTurrentAngle() {
        return turret;
    }

    public void shooterMachine(double yPos){

        telemetry.addData("turret. WE ARE HERE", turret);
        //rotateTurret(turret);


        switch (shooterState){
            case CLOSE:
                runFlywheel(getMotorVel(), 1290, 0);
                hoodPitch(0.2);
                if (gamepad2.a || yPos < 35) {
                    shooterState = FAR;
                }
                if (gamepad2.b) {
                    shooterState = OFF;
                }
                break;
            case FAR:
                telemetry.addData("turret. WE ARE HERE far", turret);
                if (gamepad2.y || yPos > 35) {
                    shooterState = CLOSE;
                }
                if (gamepad2.b) {
                    shooterState = OFF;
                }
                runFlywheel(getMotorVel(), 1610, 0);
                hoodPitch(0);
                break;
            case OFF:
                telemetry.addData("turret. WE ARE HERE not", turret);
                if (gamepad2.y) {
                    shooterState = CLOSE;
                }
                if (gamepad2.a) {
                    shooterState = FAR;
                }
                runFlywheel(getMotorVel(), 0, 0);
                hoodPitch(0.4);
                break;
        }
    }

    public void shooterMachineSingle(double yPos){
        switch (shooterState){
            case CLOSE:
                runFlywheel(getMotorVel(), 1285, 0);
                hoodPitch(0.2);

                if (gamepad1.x || yPos < 35) {
                    shooterState = FAR;
                }
                if (gamepad1.b) {
                    shooterState = OFF;
                }
                break;
            case FAR:
                if (gamepad1.a || yPos > 35) {
                    shooterState = CLOSE;
                }
                if (gamepad1.b) {
                    shooterState = OFF;
                }
                runFlywheel(getMotorVel(), 1590, 0);
                hoodPitch(0);
                break;
            case OFF:
                if (gamepad1.a) {
                    shooterState = CLOSE;
                }
                if (gamepad1.x) {
                    shooterState = FAR;
                }
                runFlywheel(getMotorVel(), 0, 0);
                hoodPitch(0.4);
                break;
        }

    }

    public enum shooterState{
        CLOSE,FAR,OFF
    }

    public void updateSimple() {
        if (gamepad2.a) {
            setVel(0.5);
        }
        if (gamepad2.b) {
            setVel(0);
        }
        telemetry.addData("We are here", "yeah");
    }
}

