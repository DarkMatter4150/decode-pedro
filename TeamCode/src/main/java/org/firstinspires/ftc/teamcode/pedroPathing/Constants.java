//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//    .mass(8.5);
//
//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            // .xVelocity(Velocity)
//            .rightFrontMotorName("fr")
//            .rightRearMotorName("br")
//            .leftRearMotorName("bl")
//            .leftFrontMotorName("fl")
//            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
//            .xVelocity()
//            .yVelocity();
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .build();
//    }
//    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
//            .leftPodY(8)
//            .rightPodY(-8)
//            .strafePodX(-7)
//            .leftEncoder_HardwareMapName("fl")
//            .rightEncoder_HardwareMapName("br")
//            .strafeEncoder_HardwareMapName("fr")
//            .leftEncoderDirection(Encoder.FORWARD)
//            .rightEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.FORWARD);
//
//
//    }
//
//
