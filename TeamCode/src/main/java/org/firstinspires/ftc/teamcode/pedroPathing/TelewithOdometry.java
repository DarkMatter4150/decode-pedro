//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//// import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
//// import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
//
//
//
//@TeleOp(name = "Teleop with Odometry", group = "Main")
//public class TelewithOdometry extends OpMode {
//    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
//    private DcMotor shooter;
//    private CRServo leftFeeder, rightFeeder;
//
//    // Pedro Pathing follower for odometry
//    private Follower follower;
//
//    // Timer for feeder control
//    private final ElapsedTime feederTimer = new ElapsedTime(); // Servo Feed Time
//
//    // Starting pose (set this to match your auto end position)
//    // Default: center of field (72, 72, 0)
//    private final Pose startPose = new Pose(72, 72, 0);
//
//    // Speed adjustment parameters
//    private final double speedMultiplier = 1.0;
//    private static final double MIN_SPEED = 0.3;  // Minimum speed multiplier
//    private static final double MAX_SPEED = 1.0;  // Maximum speed multiplier
//    private static final double SLOW_ZONE_RADIUS = 24.0; // inches - slow down within this distance
//
//    private static final double FEEDING_DURATION = 2.0;// Desired feeding Duration (in seconds)
//    private boolean feedingActive = gamepad1.left_bumper;
//
//    @Override
//    public void init() {
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//        shooter = hardwareMap.get(DcMotor.class, "launcher");
//
//        // Initialize servos
//        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
//        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
//
//        shooter.setDirection(DcMotor.Direction.FORWARD);
//
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        // Initialize Pedro Pathing follower with starting pose
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//    }
//    /**/
////    @Override
////    public void start() {
////    // Start teleop drive mode
////    follower.startTeleopDrive();
////}
//
//    @Override
//    public void loop() {
//        double axial = -gamepad1.left_stick_y * speedMultiplier;
//        double lateral = gamepad1.left_stick_x * speedMultiplier;
//        double yaw = gamepad1.right_stick_x * speedMultiplier;
//        boolean longShot = gamepad1.x;
//        double shooterPower = 0.0;
//
//
////        if (gamepad1.dpadUpWasPressed() {
////            apriltag
////        }
//
//
//        // Get current robot pose from odometry
//        Pose currentPose = follower.getPose();
//        double currentX = currentPose.getX();
//        double currentY = currentPose.getY();
//        double currentHeading = Math.toDegrees(currentPose.getHeading());
//
//        // Calculate distance from starting position
//        double distanceFromStart = Math.sqrt(Math.pow(currentX - startPose.getX(), 2) +
//                Math.pow(currentY - startPose.getY(), 2));
//
//        // Calculate wheel powers (mecanum drive)
//        double leftFrontPower = axial + lateral + yaw;
//        double rightFrontPower = axial - lateral - yaw;
//        double leftBackPower = axial - lateral + yaw;
//        double rightBackPower = axial + lateral - yaw;
//
//        // Normalize wheel powers
//        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
//                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
//
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Set motor powers
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//
//        // Set servo powers
//        leftFeeder.setPower(0.0);
//        rightFeeder.setPower(0.0);
//
//        // Feeder control
//        if (gamepad1.left_bumper) {
//            feederTimer.reset(); // Start Timing
//            feedingActive = true;
//        }
//
//        // Run feeders while active and within duration
//        if (feedingActive && feederTimer.seconds() < FEEDING_DURATION) {
//            rightFeeder.setPower(1.0);
//            leftFeeder.setPower(1.0);
//
//        } else {
//            rightFeeder.setPower(0.0);
//            leftFeeder.setPower(0.0);
//            feedingActive = false;
//
//        }
//
//        // Toggle longShot
//        if (gamepad1.x) {
//            longShot = true;
//        }
//
//        // Shooter: trigger activates, X toggles power level
//        if (gamepad1.right_trigger > 0.1) {
//            shooterPower = longShot ? 0.9 : 0.5;
//        }
//
//        shooter.setPower(shooterPower);
//
//        telemetry.addData("LF", leftFrontPower);
//        telemetry.addData("RF", rightFrontPower);
//        telemetry.addData("LB", leftBackPower);
//        telemetry.addData("RB", rightBackPower);
//        telemetry.addData("Long Shot Mode", longShot);
//
//        telemetry.addData("Feeding Active", feedingActive);
//        if (feedingActive) {
//            telemetry.addData("Time Remaining", "%.1f seconds",
//                    FEEDING_DURATION - feederTimer.seconds());
//        }
//
//        // Initialize Pedro Pathing follower with starting pose (Telemetry)
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Starting X", startPose.getX());
//        telemetry.addData("Starting Y", startPose.getY());
//        telemetry.update();
//        // Update follower to track position
//        follower.update();
//    }
//
//    @Override
//    public void stop() {
//        // Stop all motors
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        shooter.setPower(0);
//        rightFeeder.setPower(0);
//        leftFeeder.setPower(0);
//    }
//}


