/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleops.Constants;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Main Comp Teleop", group="Linear OpMode")
public class MainCompTeleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor slide = null;
    private DcMotorEx pivot = null;

    private Servo wrist = null;

    double servoSetpoint = 0;

    double pivotAfterAuto = 0;

    private Servo clawServo = null;

    /*
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    */

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        /*
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

         */

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        slide = hardwareMap.get(DcMotor.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        wrist = hardwareMap.get(Servo.class, "wrist");

        clawServo = hardwareMap.get(Servo.class, "claw");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        /*
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

         */
        slide.setDirection(DcMotor.Direction.REVERSE);
        pivot.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */


        while (!isStarted() && !isStopRequested()) {
            //telemetry.addData("pivot position: ", pivot.getCurrentPosition());
            telemetry.addData("pivot position (encoder counts):", pivot.getCurrentPosition());
            telemetry.addData("slide position (encoder counts): ", slide.getCurrentPosition());
            telemetry.addData("claw position", clawServo.getPosition());
            telemetry.update();
        }
        runtime.reset();


        if (opModeIsActive()) {
            clawServo.setPosition(1);

            wrist.setPosition(0);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleDrivetrain(drive);
            handleSlide();
            handlePivot();
            handleClaw();
            handleWrist();


            telemetry.addData("pivot position (encoder counts):", pivot.getCurrentPosition());
            telemetry.addData("slide position (encoder counts): ", slide.getCurrentPosition());
            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.addData("claw position", clawServo.getPosition());

            sleep(10);
        }
    }

    public void handleWrist() {
        servoSetpoint =  servoSetpoint + (gamepad2.right_trigger - gamepad2.left_trigger) * 0.1;

        if (gamepad2.left_bumper) { //intake specimen
            servoSetpoint = 0.6631;
        }

        else if (gamepad2.dpad_up) {
            servoSetpoint = 0.8;
        }

        else if (gamepad2.right_bumper) {
            servoSetpoint = 0.96;
        }

        servoSetpoint = Math.max(Math.min(servoSetpoint, 0.96), 0);
        telemetry.addData("servo setpoint: ", servoSetpoint);
        if (wrist.getPosition() != servoSetpoint) {
            wrist.setPosition(servoSetpoint);
        }
    }


    public void handleSlide() {
        if (gamepad2.dpad_up) { //high rung
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition((int) Constants.slide_specimen_high_rung);
            slide.setPower(1);
        }

        else if (gamepad2.dpad_down) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition((int) Constants.slide_retracted_pose);
            slide.setPower(1);
        }

        else {
            slide.setPower(-gamepad2.right_stick_y * 1.5);
        }


        /*
        if (gamepad2.dpad_up) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition((int) Constants.slide_specimen_high_rung);
            slide.setPower(1);
        } else if (gamepad2.dpad_down) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition((int) Constants.slide_retracted_pose);
            slide.setPower(1);

        }
        else if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-gamepad2.right_stick_y);
        }

         */
    }

    public void handleClaw() {

        //b is close; a is open

        if (gamepad2.a) {
            clawServo.setPosition(0.5);
        }
        else if (gamepad2.b) {
            clawServo.setPosition(1);
        }

    }

    public void handlePivot() {
        //pivot.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(Constants.pivot_p, Constants.pivot_i, Constants.pivot_d, Constants.pivot_f));
        if (gamepad2.y) {
            pivotAfterAuto = Constants.pivot_high_pose;
        }

        if (gamepad2.dpad_up) {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setTargetPosition(((int) Constants.pivot_high_pose) - (int)(pivotAfterAuto));
            pivot.setPower(1);
        } else if (gamepad2.dpad_down) {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setTargetPosition(((int) Constants.pivot_intake_pose) - (int)(pivotAfterAuto));
            pivot.setPower(1);

        }

    }




    public void handleDrivetrain(MecanumDrive drive) {
        double powMult = gamepad1.right_trigger + 0.5;

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * powMult,
                        -gamepad1.left_stick_x * powMult
                ),
                -gamepad1.right_stick_x * powMult
        ));

        /*
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

        if(gamepad1.right_trigger > 0.5) {
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * 1.5);
            rightFrontDrive.setPower(rightFrontPower * 1.5);
            leftBackDrive.setPower(leftBackPower * 1.5);
            rightBackDrive.setPower(rightBackPower * 1.5);

        }
        if(gamepad1.right_trigger <= 0.5) {
            leftFrontDrive.setPower(leftFrontPower * 0.6);
            rightFrontDrive.setPower(rightFrontPower * 0.6);
            leftBackDrive.setPower(leftBackPower * 0.6);
            rightBackDrive.setPower(rightBackPower * 0.6);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();

         */

    }
}
