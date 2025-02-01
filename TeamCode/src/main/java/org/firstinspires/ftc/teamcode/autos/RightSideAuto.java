package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.teleops.Constants;

import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="right side auto", group="beta")
public class RightSideAuto extends LinearOpMode {

    public class Intake {
        private CRServo intakeCR;
        private Servo intakeWrist;
        private Servo linkage1;
        private Servo linkage2;

        public Intake(HardwareMap hardwareMap) {
            intakeCR = hardwareMap.get(CRServo.class, "intake");
            intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
            linkage1 = hardwareMap.get(Servo.class, "linkage1");
            linkage2 = hardwareMap.get(Servo.class, "linkage2");
        }

        public class IntakeDeploy implements Action { //grab samples
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWrist.setPosition(1);
                //MAKE SURE TO MOVE PIVOT ARM UP BEFORE MOVING LINKAGES!!!
                linkage1.setPosition(0.22);
                linkage2.setPosition(0.22);

                return false;
            }
        }

        public Action intakeDeploy() {
            return new IntakeDeploy();
        }

        public class IntakeEject implements Action { //eject sample into observation zone

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWrist.setPosition(1);

                linkage1.setPosition(0.22);
                linkage2.setPosition(0.22);

                return false;
            }
        }

        public Action intakeEject() {
            return new IntakeEject();
        }

        public class IntakeStow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWrist.setPosition(0);

                linkage1.setPosition(0);
                linkage2.setPosition(0);


                return false;
            }
        }

        public Action intakeStow() {
            return new IntakeStow();
        }
    }

    public class Arm {
        private DcMotorEx armMotor;
        private Servo specWrist;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setDirection(DcMotor.Direction.FORWARD);

            specWrist = hardwareMap.get(Servo.class, "specWrist");
        }

        public class ArmSpecIntake implements Action { //intake spec pose
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(-1);
                    armMotor.setTargetPosition(-3230);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                if (Math.abs(armMotor.getCurrentPosition() - (-1800)) < 50) {
                    specWrist.setPosition(0.07);
                }

                r = r*r + 1;

                return armMotor.isBusy();

            }
        }

        public Action armSpecIntake() {
            return new ArmSpecIntake();
        }

        public class ArmReady implements Action { //move arm up ready to score
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(-1500);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (Math.abs(armMotor.getCurrentPosition() - (-1800)) < 50) {
                    specWrist.setPosition(0.7);
                }

                r = r*r + 1;
                return armMotor.isBusy();

            }
        }

        public Action armReady() {
            return new ArmReady();
        }

        public class ArmScore implements Action { //move arm to score
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specWrist.setPosition(0.7);

                if (!initialized) {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(-700);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;
                return armMotor.isBusy();

            }
        }

        public Action armScore() {
            return new ArmScore();
        }

        public class ArmUp implements Action { //for preload only!!!
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specWrist.setPosition(0.07);

                if (!initialized) {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(-1330);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;

                return armMotor.isBusy();
            }
        }

        public Action armUp() {
            return new ArmUp();
        }

        public class ArmS implements Action { //for preload only!!!!
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specWrist.setPosition(0.07);

                if (!initialized) {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(-700);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;

                return armMotor.isBusy();
            }
        }

        public Action armS() {
            return new ArmS();
        }

        public class ArmStow implements Action { //stow arm after auto
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specWrist.setPosition(0.07);

                if (!initialized) {
                    armMotor.setPower(-1);
                    armMotor.setTargetPosition(-10);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;

                return armMotor.isBusy();
            }
        }

        public Action armStow() {
            return new ArmStow();
        }

    }




    private ElapsedTime runtime = new ElapsedTime();

    private Servo claw;
    private Servo sweep;


    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class, "claw");
        sweep = hardwareMap.get(Servo.class, "sweep");

        MecanumDrive roadrunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        //INITIALIZE CLASSES HERE!!!!!
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        //DO PRE STUFF HERE (MOVE SERVOS, ETC. IF APPLICABLE!)
        claw.setPosition(0.23); //closeclaw
        sweep.setPosition(0);
        sweep.setPosition(0.3);

        //START BUILDING ACTIONS HERE
        Action preloadSpec = roadrunnerDrive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(68, 13))
                .build();

        Action pushTicks = roadrunnerDrive.actionBuilder(new Pose2d(68, 13, Math.toRadians(0))) //put arm in spec intake pose while running this
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(33, -41), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(80, -51.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(118, -66), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(91, -78.5), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(27, -80), Math.toRadians(-180))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(84, -70), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(118, -98), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(94, -108), Math.toRadians(-180))
                .setTangent(Math.toRadians(-180))
                .lineToX(4.7)
                .build();

        Action secondSpec = roadrunnerDrive.actionBuilder(new Pose2d(4.7, -108, Math.toRadians(0)))
                .waitSeconds(1)
                /*
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(70.5, 13.5), Math.toRadians(0))

                 */
                .strafeTo(new Vector2d(70, 14.5))
                .build();

        Action thirdSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(70, 14.5, Math.toRadians(0)))
                /*
                .setTangent(-180)
                .splineToConstantHeading(new Vector2d(5.45, -87), Math.toRadians(-180))

                 */
                .strafeTo(new Vector2d(5.45, -87))
                .build();



//        Action spike1 = roadrunnerDrive.actionBuilder(new Pose2d(68, 13, Math.toRadians(0)))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(41 , -44, Math.toRadians(-40)), Math.toRadians(-90))
//                .build();
//
//        Action zone1 = roadrunnerDrive.actionBuilder(new Pose2d(41, -44, Math.toRadians(-40)))
//                .turnTo(Math.toRadians(-155))
//                .build();




        //CALL ACTIONS HERE
        Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            arm.armUp(),
                            preloadSpec
                    ),
                    arm.armS(),
                    intake.intakeStow()
                )
        );

        claw.setPosition(0); //open claw

        Actions.runBlocking(
                new ParallelAction(
                        pushTicks,
                        arm.armSpecIntake()
                )
        );

        claw.setPosition(0.23);
        sleep(300);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction (
                        arm.armReady(),
                        secondSpec
                ),
                arm.armScore())
        );

        claw.setPosition(0);
        sleep(300);

        Actions.runBlocking(new ParallelAction(
                arm.armSpecIntake(),
                thirdSpecPickup
        ));

        claw.setPosition(0.23);
        sleep(300);



        sleep(10000);
//
//        Actions.runBlocking(new ParallelAction(
//                intake.intakeStow(),
//                new SequentialAction(
//                    spike1,
//                    intake.intakeDeploy()
//                )
//        ));
//
//        sleep(1800);
//
//        Actions.runBlocking(new SequentialAction(
//                zone1
//        ));
//
//        sleep(15000);
//
//
//
    }


}