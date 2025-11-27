package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(76.83625517116756)
            .yVelocity(63.33562415415846)
            .rightFrontMotorName("rightFrontMotor")
            .rightRearMotorName("rightBackMotor")
            .leftRearMotorName("leftBackMotor")
            .leftFrontMotorName("leftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //----need to make sure this is correct---
            .forwardPodY(-6.5)
            .strafePodX(0)
            //---------------------------
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static FollowerConstants followerConstants = new FollowerConstants()
            .lateralZeroPowerAcceleration(-73.66390773672457)
            .forwardZeroPowerAcceleration(-26.912657186607678)

            //PIDF
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.95, 0, 0, 0.001))

            //for dual PIDF:
            .useSecondaryTranslationalPIDF(false)//set to true for dual
            .useSecondaryHeadingPIDF(false)//set to true for dual
            .useSecondaryDrivePIDF(false)//set to true for dual
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))

            //need to change
            .mass(13.608);//need to actually weigh robot



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1)
            //.setBrakingStart(double set)
            //.setBrakingStrength(double set);
    ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
