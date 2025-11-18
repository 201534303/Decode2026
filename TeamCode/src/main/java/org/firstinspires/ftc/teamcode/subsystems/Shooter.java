package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    protected Telemetry telemetry;
    protected DcMotorEx shooterR;
    protected DcMotorEx shooterL;

    public Shooter(HardwareMap h, Telemetry t){
        shooterR = h.get(DcMotorEx.class, "shooterR");
        shooterL = h.get(DcMotorEx.class, "shooterL");
        telemetry = t;
    }

    protected void setPower(double p){
        shooterR.setPower(p);
        shooterL.setPower(-p);
    }
}
