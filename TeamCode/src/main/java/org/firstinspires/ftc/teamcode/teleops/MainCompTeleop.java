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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;


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

    private ElapsedTime runtime = new ElapsedTime();

    public Servo linkage1 = null;
    public Servo linkage2 = null;
    public double linkagePose = 0;

    public Servo specWrist = null;
    public double armPower = 0;
    public int armPose = 0;
    public Servo specClaw = null;

    public DcMotorEx armMotor = null;
    public int armTarget = 0;

    /*
    public boolean leftBump = false;
    public boolean rightBump = false;

     */

//    public Servo diffLeft = null;
//    public Servo diffRight = null;
    public Servo intakePivot = null;
    public Servo intakeClaw = null;
    public int c = 0;

    /*
    public CRServo intake = null;
    public double intakePower = 0;
    public boolean intakeOn = false;
    public boolean putPieceOut = false;
    public Servo intakeWrist = null;

     */

    public Servo sweep = null;



        @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");

//        diffLeft = hardwareMap.get(Servo.class, "diffLeft");
//        diffRight = hardwareMap.get(Servo.class, "diffRight");
//
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");


        //intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");

        specWrist = hardwareMap.get(Servo.class, "specWrist");

        //intake = hardwareMap.get(CRServo.class, "intake");

        sweep = hardwareMap.get(Servo.class, "sweep");


        specClaw = hardwareMap.get(Servo.class, "specClaw");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();





        while (!isStarted() && !isStopRequested()) {


            telemetry.update();
        }
        waitForStart();
        runtime.reset();


        //CLOCKWISE = POSITIVE MOTOR MOVEMENT IF MOTOR IS FORWARD!!!!!

            specClaw.setPosition(0);
            specWrist.setPosition(0);
            specWrist.setPosition(0.07);
            sweep.setPosition(0);
            sweep.setPosition(0.3);

           // intakeWrist.setPosition(0);

            intakeClaw.setPosition(0); //intake claw 0 = open;
            intakeClaw.setPosition(0.11); //half closed -> this will be the open position

//            diffLeft.setPosition(0);
//            diffLeft.setPosition(1);
////            diffRight.setPosition(0);
//            intakePivot.setPosition(0);
//            intakePivot.setPosition(0.28);

            linkage1.setPosition(0);
            linkage2.setPosition(0);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleDrivetrain(drive);
            handleIntake();

            //handleLinkages();
            handleArm();
            handleSweep();



            telemetry.addData("arm encoder pose", armMotor.getCurrentPosition());
            telemetry.addData("arm motor power", armMotor.getPower());
            telemetry.addData("arm pose", armPose);
            telemetry.addData("arm motor pidf", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("arm motor target pos (encoder ticks)", armMotor.getTargetPosition());
            telemetry.addData("wrist", specWrist.getPosition());
            //telemetry.addData("actual intake wrist", intakeWrist.getPosition());
            telemetry.addData("specClaw", specClaw.getPosition());

            telemetry.addData("actual linkage 1", linkage1.getPosition());
            telemetry.addData("actual linkage 2", linkage2.getPosition());
            telemetry.addData("actual sweep", sweep.getPosition());
            telemetry.addData("intake pivot", intakePivot.getPosition());
//            telemetry.addData("diffLeft", diffLeft.getPosition());
//            telemetry.addData("diffRight", diffRight.getPosition());
            telemetry.addData("intakeClaw", intakeClaw.getPosition());

            telemetry.update();

            c++;

            sleep(10);
        }
    }


    /*
    public void handleIntake() {
        if (gamepad2.right_trigger > 0.2) { //intake deploy
            putPieceOut = false;
            intakeOn = true;
        }
        else if (gamepad2.left_trigger > 0.2) { //put sample out the back
            putPieceOut = true;
            intakeOn = false;

        }
        else { //stationary
            intakeOn = false;
        }

        if (intakeOn) {
            intakePower = -1;
        }
        else if (putPieceOut) {
            if (linkagePose <= 0.05 && intakePower == 1) {
                putPieceOut = false;
            }
            if (linkage1.getPosition() <= 0.05) {
                intakePower = 1;
            }
        }
        else {
            intakePower = 0;
        }

        if (gamepad2.a) {
            intakeWrist.setPosition(1);
        }
        if (gamepad2.b) {
            intakeWrist.setPosition(0);
        }


        intake.setPower(intakePower);

    }

     */

    /*
    public void handleClaw() {
            if (gamepad2.a) { //open
                claw.setPosition(0);
            }
            else if (gamepad2.b) { //close
                claw.setPosition(0.23);
            }
    }

     */

    public void handleSweep() {
        if (gamepad2.x) { // sweep out
            sweep.setPosition(0);
        }
        else if (gamepad2.y) { //sweep in
            sweep.setPosition(0.3);
        }
    }






    public void handleArm() {
        if (gamepad2.left_bumper && armPose != 1) { //spec intake from wall
            armPose = 1;
            specClaw.setPosition(0);
            sleep(250);
        }
        else if (gamepad2.right_bumper && armPose != 2) { //mid pose
            armPose = 2;
            specClaw.setPosition(0.23);
            sleep(200);
        }
        else if (gamepad2.dpad_right && armPose == 2) { //high pose (score)
            armPose = 3;
        }
        else if (gamepad2.dpad_up || gamepad2.dpad_down) { //manual control
            armPose = 0;
        }


        if (armPose == 1) {
            specClaw.setPosition(0);
            armTarget = -3210;
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specWrist.setPosition(0.07);
        }
        else if (armPose == 2) {
            specClaw.setPosition(0.23);
            armTarget = -1600;
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (armMotor.getCurrentPosition() > -2000) {
                specWrist.setPosition(0.7);
            }
        }
        else if (armPose == 3) {
            armTarget = -700;
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specClaw.setPosition(0.23);
            specWrist.setPosition(0.7);
        }
        else if (gamepad2.dpad_down && armPose == 0) { //manual control just in case
            armTarget += 15;
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.dpad_up && armPose == 0) { //manual control just in case
            armTarget -= 15;
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (armPose == 0) {
            armTarget = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armMotor.setPower(0.5);

        if (gamepad2.dpad_left) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


    }


    public void handleIntake() {
        if (gamepad2.right_trigger >= 0.3) {
            intakePivot.setPosition(0);
        }
        else if (gamepad2.left_trigger >= 0.3) {
            intakePivot.setPosition(0.19);
        }
        //DEPLOY + RETRACT INTAKE:
        if (gamepad2.a) { //deploy
            linkage1.setPosition(0.17);
            linkage2.setPosition(0.17);
            intakePivot.setPosition(0);
            intakePivot.setPosition(0.17);

            if (c % 11 == 0) {
                intakeClaw.setPosition(0.04);
                c = 0;
            }
//            diffLeft.setPosition(0.15);
//            diffRight.setPosition(0.5);

        }
        else if (gamepad2.b) { //retract
            intakeClaw.setPosition(0.23); //CHANGE TO WHATEVER CLOSES CLAW!
//            diffLeft.setPosition(1);
//            diffRight.setPosition(0);
            if (c % 20 == 0) {
                intakePivot.setPosition(0.28);
                linkage1.setPosition(0.01);
                linkage2.setPosition(0.01);
                c = 0;
            }
        }
//
//        //ROTATE CLAW:
//        if (gamepad2.right_trigger >= 0.3 && linkage1.getPosition() > 0.17) {
//            diffLeft.setPosition(0.3);
//            diffRight.setPosition(0.7);
//        }
//        else if (gamepad2.left_trigger >= 0.3 && linkage1.getPosition() > 0.17) {
//            diffLeft.setPosition(0.15);
//            diffRight.setPosition(0.5);
//        }
    }


//    public void handleLinkages() {
//        linkagePose += -0.035*gamepad2.right_stick_y;
//        if (linkagePose <= 0) {
//            linkagePose = 0.01;
//        }
//        else if (linkagePose >= 0.248) {
//            linkagePose = 0.248;
//        }
//        /*
//        if (putPieceOut) {
//            linkagePose = 0.01;
//        }
//
//         */
//        linkage1.setPosition(linkagePose);
//        linkage2.setPosition(linkagePose);
//    }


    public void handleDrivetrain(MecanumDrive drive) {
        double powMult = 0.5 - (gamepad1.right_trigger * 0.1);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * powMult * 10,
                        -gamepad1.left_stick_x * powMult * 10
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
