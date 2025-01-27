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

@TeleOp(name="Arm Motor Test", group="Linear OpMode")
public class ArmMotorTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();



    public DcMotorEx armMotor = null;
    public double armMotorTarget = 0;
    public double armPower = 0;

    public int armPose = 0;


    public double kG = -0.01;
    public double kP2 = 0.0000001;

    public double thetaInit = Math.toRadians(-38.1176);
    public double theta = thetaInit;
    public double armError = 0;

    public double kP = 9.2; //10
    public double kI = 0.7; //1.25
    public double kD = 1.7; //2.5
    public double kF = 0;

    PIDFCoefficients pidfCoeff = new PIDFCoefficients(kP, kI, kD, kF);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.





        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
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
        runtime.reset();

        /*
        if (opModeIsActive()) {
            clawServo.setPosition(1);

            //wrist.setPosition(0);
        }

        */

        //CLOCKWISE = POSITIVE MOTOR MOVEMENT IF MOTOR IS FORWARD!!!!!


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //armMotor.setPower(gamepad2.left_stick_y);



            if (gamepad2.left_bumper && armPose != 1) {
                armPose = 1;
            }
            else if (gamepad2.right_bumper && armPose == 1) {
                armPose = 2;
            }
            else if (gamepad2.right_bumper && armPose == 2 && Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) < 30) {
                armPose = 3;
            }

            if (armPose == 1) {
                armMotor.setTargetPosition(-325);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
                //do stuff with claw and wrist
            }
            else if (armPose == 2) {
                armMotor.setTargetPosition(-240);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
                //do stuff with claw and wrist
            }
            else if (armPose == 3) {
                armMotor.setTargetPosition(-100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
                //do stuff with claw and wrist
            }
            else {
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
            }

            armMotor.setPower(-0.5);




            /*
            //base PID
            if (gamepad2.left_bumper) {
                armMotorTarget = -325;
                armError = armMotorTarget - armMotor.getCurrentPosition();
                armPower = kP2 * armError * Math.abs(armError) + kG * Math.cos(theta) * Math.cos(theta);
                armMotor.setPower(armPower);
            }
            else {
                armMotor.setPower(kG * Math.cos(theta));
            }

            armError = armMotorTarget - armMotor.getCurrentPosition();


            theta = thetaInit + (-1.0 * Math.PI * armMotor.getCurrentPosition() / 255);



            telemetry.addData("arm angle degrees", Math.toDegrees(theta));

            telemetry.addData("arm error", armError);

             */

            telemetry.addData("arm motor power", armMotor.getPower());

            telemetry.addData("arm encoder pose", armMotor.getCurrentPosition());

            telemetry.addData("arm motor pidf", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("arm motor target pos", armMotor.getTargetPosition());
            telemetry.addData("arm motor", armMotor.getCurrentPosition());


            telemetry.addData("arm target", armMotorTarget);
            telemetry.update();

        }
    }



}
