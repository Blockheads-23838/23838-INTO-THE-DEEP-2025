package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.teleops.Constants;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="right side auto", group="beta")
public class RightSideAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor slide = null;
    private DcMotorEx pivot = null;

    private Servo wrist = null;

    double servoSetpoint = 0;

    private Servo clawServo = null;

    private boolean boolPivotUp = false;

    //move robot 28 inches forward, score on high rung, open claw, bring slide back, move robot back, pivot down, then push 3 pieces in.



    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotor.class, "slide");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        wrist = hardwareMap.get(Servo.class, "wrist");

        clawServo = hardwareMap.get(Servo.class, "claw");

        MecanumDrive roadrunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(9, -63, Math.toRadians(90)));


        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.REVERSE);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runtime.reset();

        Action toSub = roadrunnerDrive.actionBuilder(new Pose2d(9, -63, Math.toRadians(90)))
                .lineToY(-3)
                .build();


        Action push3Ticks = roadrunnerDrive.actionBuilder(new Pose2d(9, -3, Math.toRadians(90)))
                .lineToY(-5)
                .splineToLinearHeading(new Pose2d(78, -17, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                //.strafeTo(new Vector2d(63, -20))
                .strafeTo(new Vector2d(63, 43))
                .strafeTo(new Vector2d(83, 43))
                .strafeTo(new Vector2d(83, -38))
                .strafeTo(new Vector2d(83, 43))
                .strafeTo(new Vector2d(103, 43))
                .strafeTo(new Vector2d(103, -38))
                .strafeTo(new Vector2d(103, 43))
                .strafeTo(new Vector2d(117, 43))
                .strafeTo(new Vector2d(117, -38))
                //.splineToLinearHeading(new Pose2d(80, 40, Math.toRadians(-90)), Math.toRadians(90))
                //.setReversed(true)
                //.strafeTo(new Vector2d(117, 40))
                //.strafeTo(new Vector2d(117, -38))

                .build();

        Action secondSpec = roadrunnerDrive.actionBuilder(new Pose2d(117, -38, Math.toRadians(-90)))
                .setReversed(true)
                .strafeTo(new Vector2d(90, -50))
                .strafeTo(new Vector2d(90, -58.29))
                //.splineToLinearHeading(new Pose2d(90, -60, Math.toRadians(-90)), Math.toRadians(90))
                /*
                .strafeTo(new Vector2d(110, -60))
                .splineTo(new Vector2d(100, -50), Math.toRadians(90))
                .splineTo(new Vector2d(90, -60), Math.toRadians(180))

                 */
                //.strafeTo(new Vector2d(90, -48))
                //.strafeTo(new Vector2d(95, -60))
                //.setReversed(true)
                //.strafeTo(new Vector2d(90, -60))
                .build();

        Action secondToSub = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.29, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-10, -40, Math.toRadians(90)), Math.toRadians(-90))
                .setReversed(false)
                .strafeTo(new Vector2d(-6, -3))
                .build();

        Action thirdSpec = roadrunnerDrive.actionBuilder(new Pose2d(-6, -3, Math.toRadians(90)))
                .strafeTo(new Vector2d(-5, -15))
                .splineToSplineHeading(new Pose2d(100, -40, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                .strafeTo(new Vector2d(90, -58.4))
                //.splineToLinearHeading(new Pose2d(90, -60, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        Action thirdToSub = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.4, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-15, -40, Math.toRadians(90)), Math.toRadians(-90))
                .setReversed(false)
                .strafeTo(new Vector2d(-12, -3))
                .build();


        //auto commands start here
        closeClaw();
        Actions.runBlocking(toSub);
        pivotUpOne();
        wristScoreSpecimen1();
        slideOutOne();
        sleep(200);
        openClaw();
        slideIn();
        pivotDown();
        Actions.runBlocking(push3Ticks);
        //pivotDown();
        Actions.runBlocking(secondSpec);
        //wristIntakeSpecimen();
        closeClaw();
        sleep(200);
        pivotUpTwo();
        Actions.runBlocking(secondToSub);
        wristScoreSpecimen2();
        slideOutTwo();
        sleep(300);
        openClaw();
        slideIn();
        pivotDown();
        Actions.runBlocking(thirdSpec);
        wristScoreSpecimen1(); //works better as an intake pose
        closeClaw();
        sleep(200);
        pivotUpTwo();
        Actions.runBlocking(thirdToSub);
        wristScoreSpecimen2();
        slideOutTwo();
        sleep(300);
        openClaw();
        slideIn();
        pivotDown();




        /*
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleSlide();
            handlePivot();
            handleWrist();


            telemetry.addData("pivot position (encoder counts):", pivot.getCurrentPosition());
            telemetry.addData("slide position (encoder counts): ", slide.getCurrentPosition());
            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.addData("claw position", clawServo.getPosition());

            sleep(10);
        }
        */


    }



    public void wristIntakeSpecimen() {
       wrist.setPosition(0.4);
    }

    public void wristScoreSpecimen1() {
        wrist.setPosition(0.45);
    }

    public void wristScoreSpecimen2() {
        wrist.setPosition(0.38);
    }

    public void slideIn() {
        slide.setTargetPosition((int) Constants.slide_retracted_pose);
        slide.setPower(1);
        int r = 0;
        while (slide.isBusy()) {
            r = r*r + 1;
        }
    }

    public void slideOutOne() {
        //slide.setPower(-gamepad2.right_stick_y * 1.5);

        slide.setTargetPosition((int) Constants.slide_specimen_high_rung);
        slide.setPower(1);
        int r = 0;
        while (slide.isBusy()) {
            r = r*r+1;
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

    public void slideOutTwo() {
        //slide.setPower(-gamepad2.right_stick_y * 1.5);

        slide.setTargetPosition((int) Constants.slide_specimen_high_rung_2);
        slide.setPower(1);
        int r = 0;
        while (slide.isBusy()) {
            r = r * r + 1;
        }

    }

    public void openClaw() {
        clawServo.setPosition(0.5);
    }

    public void closeClaw() {
        clawServo.setPosition(1);
    }

    public void pivotUpOne() {
        pivot.setTargetPosition((int) Constants.pivot_high_pose_auto_1);
        pivot.setPower(1);
        int r = 0;
        while (pivot.isBusy()) {
           r = r*r + 1;
        }
    }

    public void pivotUpTwo() {
        pivot.setTargetPosition((int) Constants.pivot_high_pose_auto_2);
        pivot.setPower(1);
        int r = 0;
        while (pivot.isBusy()) {
            r = r*r + 1;
        }
    }

    public void pivotDown() {
        pivot.setTargetPosition((int) Constants.pivot_intake_pose);
        pivot.setPower(1);
        int r = 0;
        while (pivot.isBusy()) {
            r =r*r+1;
        }
    }

    /*
    public void handlePivot() {

        if (boolPivotUp) {
            while (pivot.getCurrentPosition() < Constants.pivot_high_pose) {
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setTargetPosition((int) Constants.pivot_high_pose);
                pivot.setPower(1);
            }

        } else {
            while (pivot.getCurrentPosition() > Constants.pivot_intake_pose) {
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot.setTargetPosition((int) Constants.pivot_intake_pose);
                pivot.setPower(1);
            }
        }

    }
    */

}
