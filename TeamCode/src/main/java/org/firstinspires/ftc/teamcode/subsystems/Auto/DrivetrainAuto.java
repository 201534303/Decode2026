package org.firstinspires.ftc.teamcode.subsystems.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DrivetrainAuto extends Drivetrain {
    public DrivetrainAuto(HardwareMap hardwareMap, Telemetry t) {
        super(hardwareMap, t);
    }

    public void stopRobot(){
        driveRobot(0,0,0);
    }


}
