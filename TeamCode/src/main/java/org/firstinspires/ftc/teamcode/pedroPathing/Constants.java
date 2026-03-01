package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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

    // ---------- Follower Constants ----------
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.55)
            .forwardZeroPowerAcceleration(-64.96374694621761)
            .lateralZeroPowerAcceleration(-24.060439884905)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.005))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0.0006))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0.0, 0.05, 0.0))
            // Reduced from 1.0 to 0.7 to stop the jitter as the robot 'parks' at its final heading
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.08, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0.0, 0.0001, 0.5, 0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.000005, 0.5, 0.01))
            .drivePIDFSwitch(15)
            .translationalPIDFSwitch(2)
            .centripetalScaling(0.0003);

    // ---------- Mecanum Drive ----------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.935)
            .leftFrontMotorName("front_left_drive")
            .leftRearMotorName("back_left_drive")
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(69.3422928457185)
            .yVelocity(58.49372443251724);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.08661)
            .strafePodX(-2.51969)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ---------- Path Constraints ----------
    public static PathConstraints pathConstraints = new PathConstraints
            (0.995, 99, 0.97, 0.93);

    // ---------- Follower Builder ----------
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}