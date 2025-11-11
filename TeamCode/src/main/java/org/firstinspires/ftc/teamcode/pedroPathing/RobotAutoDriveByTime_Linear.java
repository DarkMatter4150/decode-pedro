/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class RobotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor shooter;
    private CRServo leftFeeder, rightFeeder;


    private final ElapsedTime runtime = new ElapsedTime(); // Servo Feed Time
    private final ElapsedTime rpmTimer = new ElapsedTime(); // RPM timer

    // RPM Calculation Variables
    private int previousEncoderPosition = 0;
    public double currentRPM = 0;

    // Constants
    // static final double SHOOTER_SPINUP_TIME = 0.75; // Seconds to reach full speed

    private static final double TARGET_RPM = 2750; // Adjust to you shooter's speed
    private static final double RPM_TOLERANCE = 100; // +- 100 RPM is acceptable
    static final double SHOOTER_LONG_POWER = 0.8; // Power for long shot

    // static final double SHOOTER_SHORT_POWER = 0.5; // Power for short shot (if needed)

    static final double FEEDER_POWER = 1.0; // Feeder motor power
    private static final double FEEDING_DURATION = 0.35; // Desired feeding Duration (in seconds)
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    // Motor spec (REV HD Hex Motor example - adjust your motor)
    private static final double TICKS_PER_REVOLUTION = 28; // Encoder ticks per motor
    private static final double GEAR_RATIO = 1.0; // Adjust if geared (Output/Input)
    private static final double MIN_TIME_ELAPSED = .01; // Prevent div-by-zero in RPM calc


    @Override
    public void runOpMode() {

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
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Configured shooter motor encoder
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", " RPM Shooting - Initialized and Ready");
        telemetry.addData(">", "Press START to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step 1: Drive forward for 3 seconds
//        telemetry.addData("Status", "Step 1: Driving Forward");
//        telemetry.update();
//
//        leftFrontDrive.setPower(DRIVE_SPEED);
//        rightFrontDrive.setPower(DRIVE_SPEED);
//        leftBackDrive.setPower(DRIVE_SPEED);
//        rightBackDrive.setPower(DRIVE_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 2: Spin left for 1.3 seconds
//        telemetry.addData("Status", "Step 2: Spinning Right");
//        telemetry.update();
//
//        leftFrontDrive.setPower(-TURN_SPEED);
//        rightFrontDrive.setPower(TURN_SPEED);
//        leftBackDrive.setPower(-TURN_SPEED);
//        rightBackDrive.setPower(TURN_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        // Step 3:  Drive Backward for 0.5 Seconds
//        telemetry.addData("Status", "Step 3: Driving Backward");
//        telemetry.update();
//
//        leftFrontDrive.setPower(-FORWARD_SPEED);
//        rightFrontDrive.setPower(-REVERSE_SPEED);
//        leftBackDrive.setPower(-FORWARD_SPEED);
//        rightBackDrive.setPower(-REVERSE_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//
//        }

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // ===== AUTONOMOUS SEQUENCE =====


        // Step "1": STOP and prepare to shoot
        telemetry.addData("Status", "Step 1: Stopping for Shot");
        telemetry.update();

        stopDrive();
        sleep(250); // Brief pause to stabilize



        // Step "2": SHOOT (Long Shot Mode)
        telemetry.addData("Status", "Step 2: Say Hello to my LITTLE friend!");
        telemetry.update();

        // Shoot 3 artifacts with RPM monitoring
        for (int i = 1; i <= 3; i++) {
            telemetry.addData("Status", "Shooting artifact " + i + " of 3");
            telemetry.update();

            shooter.setPower(SHOOTER_LONG_POWER);  // Long shot power (same as teleop X button)
            sleep(200); // Short delay for initial acceleration before RPM calc
            waitForTargetRPM();  // Wait for shooter to reach full speed (adjust as needed)

            // Feed artifact once RPM is stable
            feedArtifact(i);

            // Pause between shots
            sleep(1000);

        }

        // Stop shooter after all shots
        shooter.setPower(0);

        telemetry.addData("Status", "All 3 shots complete!");
        telemetry.update();
        sleep(500);



        // Step "3": Drive forward for 1.0 seconds
        driveForward(1.0);

        // Final Stop
        stopDrive();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

//
//        // Step 7: Spin right for 1.3 seconds
//        telemetry.addData("Status", "Step 7: Spinning Right");
//        telemetry.update();
//
//        leftFrontDrive.setPower(TURN_SPEED);
//        rightFrontDrive.setPower(-TURN_SPEED);
//        leftBackDrive.setPower(TURN_SPEED);
//        rightBackDrive.setPower(-TURN_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 8:  Drive Backward for 3.0 Seconds
//        telemetry.addData("Status", "Step 8: Driving Backward");
//        telemetry.update();
//
//        leftFrontDrive.setPower(-FORWARD_SPEED);
//        rightFrontDrive.setPower(-REVERSE_SPEED);
//        leftBackDrive.setPower(-FORWARD_SPEED);
//        rightBackDrive.setPower(-REVERSE_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
//
//        }



    /**
     * Method to wait until shooter reaches target RPM before feeding
     */
    private void waitForTargetRPM() {
        rpmTimer.reset();
        previousEncoderPosition = shooter.getCurrentPosition();

        telemetry.addData("Status", "Spinning up shooter...");
        telemetry.update();

        while (opModeIsActive()) {
            // Calculate RPM
            currentRPM = calculateRPM();

            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Status", currentRPM >= (TARGET_RPM - RPM_TOLERANCE) ?
                    "READY TO FEED" : "Spinning up...");
            telemetry.update();

            // Exit when RPM is within tolerance
            if (currentRPM >= (TARGET_RPM - RPM_TOLERANCE)) {
                break;
            }

            // Safety timeout (max 5 seconds to reach speed)
            if (rpmTimer.seconds() > 5.0) {
                telemetry.addData("Warning", "RPM timeout - feeding anyway");
                telemetry.update();
                break;
            }

            sleep(50);  // Check every 50ms
        }
    }

    /**
     * Method to calculate current RPM of shooter wheel
     */
    private double calculateRPM() {
        // Get time elapsed since last calculation
        double timeElapsed = rpmTimer.seconds();

        // Safeguard against div-by-zero on first/short intervals
        if (timeElapsed < MIN_TIME_ELAPSED) {
            rpmTimer.reset();  // Reset and wait longer next time
            return 0.0;
        }

        // Get current encoder position
        int currentPosition = shooter.getCurrentPosition();

        // Calculate ticks traveled
        int ticksTravel = currentPosition - previousEncoderPosition;

        // Calculate revolutions
        double revolutions = ticksTravel / (TICKS_PER_REVOLUTION * GEAR_RATIO);
        // Calculate RPM (revolutions per minute)
        double rpm = (revolutions / timeElapsed) * 60.0;

        // Update for next calculation
        previousEncoderPosition = currentPosition;
        rpmTimer.reset();

        return Math.abs(rpm);  // Return absolute value
    }
    /**
     * Method to feed a single artifact
     */
    private void feedArtifact(int shotNumber) {
        telemetry.addData("Status", "FEEDING shot " + shotNumber);
        telemetry.update();

        rightFeeder.setPower(FEEDER_POWER);
        leftFeeder.setPower(FEEDER_POWER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < FEEDING_DURATION)) {
            // Continue monitoring RPM while feeding
            currentRPM = calculateRPM();

            telemetry.addData("Shot", shotNumber + " of 3");
            telemetry.addData("Feeding", "%.1f / %.1f seconds",
                    runtime.seconds(), FEEDING_DURATION);

            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.update();
            sleep(50);
        }

        // Stop feeders
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

    }

    /**
     * Method to Drive forward for specified time
     */
    private void driveForward(double seconds) {
        telemetry.addData("Status", "Driving Forward");
        telemetry.update();

        leftFrontDrive.setPower(DRIVE_SPEED);
        rightFrontDrive.setPower(DRIVE_SPEED);
        leftBackDrive.setPower(DRIVE_SPEED);
        rightBackDrive.setPower(DRIVE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Driving: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        stopDrive();
    }


    /**
     * Method to stop all drive motors
     */
    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
}


