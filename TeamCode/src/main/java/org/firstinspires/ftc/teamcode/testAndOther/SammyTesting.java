package org.firstinspires.ftc.teamcode.testAndOther;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Config.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths.OLD.OLDClosePaths;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.DrivetrainTele;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.IntakeTele;
import org.firstinspires.ftc.teamcode.subsystems.OldTele.ShooterTele;
import org.firstinspires.ftc.teamcode.subsystems.superClasses.Shooter;

@Config
@TeleOp(name = "Sammy Shooter Testing")
@Disabled

public class SammyTesting extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Servo hood;
    MotorEx shooterR;
    MotorEx shooterL;
    double last_error = 0;
    double integral = 0;
    int speed = 1500;
    boolean pressed = false;
    boolean pressed2 = false;
    public static double x;
    public static double g;
    public static double a;
    public static double highH;
    public static double lowH;
    private double sammyEquation (double x, double g, double a, double highH, double lowH) {
        double s1 = Math.cos(a);
        double s2 = 2 * Math.pow(s1, 2);
        double s3 = (x * Math.tan(a)) - (highH - lowH);
        double s4 = s2 * s3;
        double s5 = g / s4;
        return x * s5;

        //delta h height of
        //x horizontal direct distance to the goal
    }

    public void init() {
        shooterR = new MotorEx(hardwareMap, "shooterR", MotorEx.GoBILDA.BARE);
        shooterL = new MotorEx(hardwareMap, "shooterL", MotorEx.GoBILDA.BARE);
        shooterR.setInverted(true);

        shooterR.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterL.setRunMode(MotorEx.RunMode.VelocityControl);

        telemetry.addData("INIT", "DONE");
        telemetry.update();
    }

    public void loop() {
        //double flywheelSpeed = sammyEquation(x, g, a, highH, lowH);

        if (gamepad1.dpad_up && !pressed) {
            speed += 5;
            pressed = true;
        } else if (!gamepad1.dpad_up) {
            pressed = false;
        }
        if (gamepad1.dpad_down && !pressed2) {
            speed -= 5;
            pressed2 = true;
        } else if (!gamepad1.dpad_down) {
            pressed2 = false;
        }

        flywheelSpin(speed, getMotorVel(), 0);

        //telemetry.addData("equation:", flywheelSpeed);
        telemetry.addData("flywheel speed (rpm)", getMotorVel());
        telemetry.addData("speed", speed);
        telemetry.update();
    }

    public double getMotorVel(){
        //in ticks/sec - gobilda bare is 28 tps
        return shooterL.getVelocity();
    }

    public double getMotorRPM(){
        //gets rotations per sec then converts it to rpm
        return (shooterL.getVelocity()/28)*60;
    }

    public double RPMToVel(double RPM){
        return (RPM/60)*28;
    }

    public void setVel(double flywheelV){
        shooterR.setVelocity(flywheelV);
        shooterL.setVelocity(flywheelV);
    }

    protected void setRPM(double flywheelRPM){
        shooterR.setVelocity(RPMToVel(flywheelRPM));
        shooterL.setVelocity(RPMToVel(flywheelRPM));
    }

    public void flywheelSpin(double targetVelo, double currentVelo, double kf){//kf is a tester varible
        double speed = PIDF(targetVelo-currentVelo, targetVelo, 12,0,0.1,0.59);
        shooterR.setVelocity(speed);
        shooterL.setVelocity(speed);
        telemetry.addData("target velocity", Math.round(targetVelo*100)/100.0);
        telemetry.addData("current velocity", Math.round(currentVelo*100)/100.0);
    }

    public double PIDF(double error, double setpoint, double kp, double ki, double kd, double kF) {

        integral += error;
        double derivative = error - last_error;

        double proportional = error * kp;
        double integralTerm = integral * ki;
        double derivativeTerm = derivative * kd;

        // Feedforward = kF * setpoint; important for scaling feedforward
        double feedforward = kF * setpoint;

        double correction = proportional + integralTerm + derivativeTerm + feedforward;

        last_error = error;

        return correction;
    }
}
