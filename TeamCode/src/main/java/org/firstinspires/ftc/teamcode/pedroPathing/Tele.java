package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp()
public class Tele extends OpMode {
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor shooter;
    private CRServo leftFeeder, rightFeeder;


    private final ElapsedTime feederTimer = new ElapsedTime(); // Servo Feed Time

    private static final double FEEDING_DURATION = 1.5; // Desired feeding Duration (in seconds)

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        shooter = hardwareMap.get(DcMotor.class, "launcher");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");

        shooter.setDirection(DcMotor.Direction.FORWARD);

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }
/**/
    @Override
    public void loop() {
        rightFeeder.setPower(0);
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        boolean longShot = false;
        boolean feedingActive = false;
        double shooterPower;

        // Left Bumper Init Feeders
        if (gamepad1.left_bumper) {
            feederTimer.reset(); // Start Timing
            feedingActive = true;
        }

        // Run feeders while active and within duration
        if (feedingActive && feederTimer.seconds() < FEEDING_DURATION) {
            rightFeeder.setPower(1);
            leftFeeder.setPower(1);

        } else {
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
            feedingActive = false;

        }



//        if (gamepad1.dpadUpWasPressed() {
//            apriltag
//        }

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));


        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Toggle longShot
        if (gamepad1.x) {
            longShot = true;
        }

        // Shooter: trigger activates, X toggles power level
        if (gamepad1.right_trigger > 0.1) {
            shooterPower = longShot ? 0.8 : 0.5;
        } else {
            shooterPower = 0.0;
        }
        shooter.setPower(shooterPower);

        telemetry.addData("LF", leftFrontPower);
        telemetry.addData("RF", rightFrontPower);
        telemetry.addData("LB", leftBackPower);
        telemetry.addData("RB", rightBackPower);
        telemetry.addData("Long Shot Mode", longShot);

        telemetry.addData("Feeding Active", feedingActive);
        if (feedingActive) {
            telemetry.addData("Time Remaining", "%.1f seconds",
                    FEEDING_DURATION - feederTimer.seconds());
        }
        telemetry.update();
    }
}