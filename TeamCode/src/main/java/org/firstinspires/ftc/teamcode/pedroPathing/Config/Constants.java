package org.firstinspires.ftc.teamcode.pedroPathing.Config;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            //.xVelocity(76.83625517116756)
            //.yVelocity(63.33562415415846)
            .xVelocity(72.79995319036048)//new
            .yVelocity(54.70373439037894)//new
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
            //center of robot
                //.forwardPodY(-6.5)
                //.strafePodX(0)
            //center relative to shooter
            .forwardPodY(-5.5)
            .strafePodX(-5)
            //---------------------------
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static FollowerConstants followerConstants = new FollowerConstants()
            //.lateralZeroPowerAcceleration(-73.66390773672457)
            //.forwardZeroPowerAcceleration(-26.912657186607678)
            .forwardZeroPowerAcceleration(-35.71563085866153)//new
            .lateralZeroPowerAcceleration(-81.38187788469911)//new

            //PIDF
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0, 0.001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.019,0.0,0.00001,0.6,0.01))
            .centripetalScaling(0.0006)

            //for dual PIDF:
            .useSecondaryTranslationalPIDF(true)//set to true for dual
            .useSecondaryHeadingPIDF(true)//set to true for dual
            .useSecondaryDrivePIDF(true)//set to true for dual
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.02,0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.6,0,0.2,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.021,0,0.000005,0.6,0.01))

            //need to change
            .mass(13.608);//need to actually weigh robot



    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.1,
            1)
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