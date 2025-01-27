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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    /*
    public CRServo arm1 = null; //closer side to spec claw
    public CRServo arm2 = null; //closer side to sample intake

     */
    public Servo specWrist = null;
    public double armPower = 0;
    public double specWristPose = 0;
    public enum armPose {
        specIntake,
        mid,
        score,
        none
    };
    armPose arm = armPose.none;
    public Servo claw = null;
    public double clawPose = 0;

    public DcMotorEx armMotor = null;
    public double armMotorTarget = 0;

    public boolean leftBump = false;
    public boolean rightBump = false;

    public CRServo intake = null;
    public double intakePower = 0;
    public boolean intakeOn = false;
    public boolean putPieceOut = false;
    public double intakeWristPose = 0;
    public Servo intakeWrist = null;

    public double t;


    public double r = 0;

    public double kP = 0.00001;
    public double kI = 0.4;
    public double kD = 8;
    public double kF = 0;

    public double kG = -0.01;

    public double thetaInit = Math.toRadians(-38.1176);
    public double theta = thetaInit;
    public double armError = 0;

    public Servo sweep = null;

    PIDFCoefficients pidfCoeff = new PIDFCoefficients(kP, kI, kD, kF);


        @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        linkage1 = hardwareMap.get(Servo.class, "linkage1");
        linkage2 = hardwareMap.get(Servo.class, "linkage2");


        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");

        specWrist = hardwareMap.get(Servo.class, "specWrist");

        intake = hardwareMap.get(CRServo.class, "intake");

        sweep = hardwareMap.get(Servo.class, "sweep");

        /*
        arm1 = hardwareMap.get(CRServo.class, "arm1");
        arm2 = hardwareMap.get(CRServo.class, "arm2");
         */

        claw = hardwareMap.get(Servo.class, "claw");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor"); //TEST JUST IN CASE WE NEED TO SWITCH ARM TO MOTOR!
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        //armMotor.setTargetPosition(armMotor.getCurrentPosition());


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




        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();





        while (!isStarted() && !isStopRequested()) {


            telemetry.update();
        }
        waitForStart();
        runtime.reset();

        /*
        if (opModeIsActive()) {
            clawServo.setPosition(1);

            //wrist.setPosition(0);
        }

        */

            //CLOCKWISE = POSITIVE MOTOR MOVEMENT IF MOTOR IS FORWARD!!!!!

            specWrist.setPosition(0);
            sweep.setPosition(0);
            sweep.setPosition(0.3);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleDrivetrain(drive);
            handleIntake();
            handleLinkages();
            //handleArm();
            handleSweep();
            //armMotor.setPower(gamepad2.left_stick_y);

            if (arm == armPose.mid) {
                specWrist.setPosition(0.63);
            }
            else if (arm == armPose.specIntake) {
                specWrist.setPosition(0);
            }


            /*
            if (gamepad2.left_bumper) {
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setTargetPosition(-325);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else if (gamepad2.right_bumper) {
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setTargetPosition(-240);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

             */

            //base PID
            if (gamepad2.left_bumper) {
                armMotorTarget = -325; //just trying -250 for now.
                armError = armMotorTarget - armMotor.getCurrentPosition();
                armPower = kP * armError + kG * Math.cos(theta);
                armMotor.setPower(armPower);
            }
            else {
                armMotor.setPower(kG * Math.cos(theta));
            }

            armError = armMotorTarget - armMotor.getCurrentPosition();



            //armMotor.setPower(1);

            theta = thetaInit + (-1.0 * Math.PI * armMotor.getCurrentPosition() / 255);





            telemetry.addData("arm angle degrees", Math.toDegrees(theta));
            telemetry.addData("arm power calculated", armPower);
            telemetry.addData("arm encoder pose", armMotor.getCurrentPosition());
            telemetry.addData("arm error", armError);

            telemetry.addData("arm motor power", armMotor.getPower());
            telemetry.addData("actual linkage 1", linkage1.getPosition());
            telemetry.addData("actual linkage 2", linkage2.getPosition());
            telemetry.addData("wrist", specWrist.getPosition());
            telemetry.addData("arm pose enum", arm);
            /*
            telemetry.addData("arm motor pidf", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("arm motor target pos", armMotor.getTargetPosition());
            telemetry.addData("arm motor", armMotor.getCurrentPosition());

             */
            telemetry.addData("arm target", armMotorTarget);
            telemetry.update();

            sleep(10);

            r++;
        }
    }



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
            //intakeWristPose = 1;
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
            //intakeWristPose = 0;
        }

        if (gamepad2.a) {
            intakeWristPose = 1;
        }
        else if (gamepad2.b) {
            intakeWristPose = 0;
        }


        intake.setPower(intakePower);
        intakeWrist.setPosition(intakeWristPose);

    }

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
        if (gamepad2.x) {
            sweep.setPosition(0);
        }
        else if (gamepad2.y) {
            sweep.setPosition(0.3);
        }
    }





    public void handleArm() {
        if (gamepad2.left_bumper && !leftBump) {
            leftBump = true;
            arm = armPose.specIntake;
            claw.setPosition(0);
//            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
            armMotor.setTargetPosition(-337);
//            armMotor.setTargetPosition(-190);

            //specWrist.setPosition(0);
        }
        else if (!gamepad2.left_bumper && leftBump && !armMotor.isBusy()) {
            leftBump = false;
        }

        if (gamepad2.right_bumper && !rightBump && arm == armPose.specIntake) {
            rightBump = true;
            arm = armPose.mid;
            claw.setPosition(0.23);
//            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
            armMotor.setTargetPosition(-140);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(0.5);
            //specWrist.setPosition(0.63);
        }
        else if (!gamepad2.right_bumper && rightBump && !armMotor.isBusy()) {
            rightBump = false;
        }

        if (gamepad2.right_bumper && !rightBump && arm == armPose.mid) {
            rightBump = true;
            arm = armPose.score;
//            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
            armMotor.setTargetPosition(-96);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(0.5);

        }

        else if (!gamepad2.right_bumper && rightBump && !armMotor.isBusy()) {
            rightBump = false;
        }


    }


    public void handleLinkages() {
        linkagePose += -0.035*gamepad2.right_stick_y;
        if (linkagePose <= 0) {
            linkagePose = 0.01;
        }
        else if (linkagePose >= 0.2) {
            linkagePose = 0.2;
        }

        if (putPieceOut) {
            linkagePose = 0.01;
        }
        linkage1.setPosition(linkagePose);
        linkage2.setPosition(linkagePose);
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
