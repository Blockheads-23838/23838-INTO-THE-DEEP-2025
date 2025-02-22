package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="right side auto", group="beta")
public class RightSideAuto extends LinearOpMode {

    public class Intake {
//        private CRServo intakeCR;
//        private Servo intakeWrist;
        public Servo intakeClaw;
        public Servo intakePivot;
        private Servo linkage1;
        private Servo linkage2;

        public Intake(HardwareMap hardwareMap) {
//            intakeCR = hardwareMap.get(CRServo.class, "intake");
//            intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
            intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
            intakePivot = hardwareMap.get(Servo.class, "intakePivot");
            linkage1 = hardwareMap.get(Servo.class, "linkage1");
            linkage2 = hardwareMap.get(Servo.class, "linkage2");
        }

        public class IntakeDeploy implements Action { //grab samples
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //intakeWrist.setPosition(1);
                //MAKE SURE TO MOVE PIVOT ARM UP BEFORE MOVING LINKAGES!!!

                linkage1.setPosition(0.17);
                linkage2.setPosition(0.17);
                intakeClaw.setPosition(0.04);
                intakePivot.setPosition(0.009);

                return false;
            }
        }

        public Action intakeDeploy() {
            return new IntakeDeploy();
        }

        public class IntakeGrab implements Action { //grab sample

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkage1.setPosition(0.17);
                linkage2.setPosition(0.17);
                intakeClaw.setPosition(0.23);
                intakePivot.setPosition(0.009);

                return false;
            }
        }

        public Action intakeGrab() {
            return new IntakeGrab();
        }

        public class IntakeUp implements Action { //intake pivot parallel to ground
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkage1.setPosition(0.17);
                linkage2.setPosition(0.17);
                intakePivot.setPosition(0.17);
                intakeClaw.setPosition(0.23);

                return false;
            }
        }

        public Action intakeUp() {
            return new IntakeUp();
        }

        public class IntakeStow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeClaw.setPosition(0);
                intakePivot.setPosition(0);
                intakePivot.setPosition(0.17);

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
                    armMotor.setTargetPosition(-3143);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                if (Math.abs(armMotor.getCurrentPosition() - (-1800)) < 50) {
                    specWrist.setPosition(0.07);
                }

                //r = r*r + 1;

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

                //r = r*r + 1;
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
                    armMotor.setTargetPosition(-670);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                //r = r*r + 1;
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
                    armMotor.setTargetPosition(-1400);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                //r = r*r + 1;

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
                    armMotor.setTargetPosition(-665);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                //r = r*r + 1;

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
                    armMotor.setTargetPosition(-5);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                //r = r*r + 1;

                return armMotor.isBusy();
            }
        }

        public Action armStow() {
            return new ArmStow();
        }

    }




    private ElapsedTime runtime = new ElapsedTime();

    private Servo specClaw;
    private Servo sweep;


    @Override
    public void runOpMode() throws InterruptedException {

        specClaw = hardwareMap.get(Servo.class, "specClaw");
        sweep = hardwareMap.get(Servo.class, "sweep");

        MecanumDrive roadrunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        //INITIALIZE CLASSES HERE!!!!!
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        //DO PRE STUFF HERE (MOVE SERVOS, ETC. IF APPLICABLE!)
        specClaw.setPosition(0.23); //closeclaw
        sweep.setPosition(0);
        sweep.setPosition(0.3);

        //START BUILDING ACTIONS HERE
        Action preloadSpec = roadrunnerDrive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(40, -5))
                .strafeTo(new Vector2d(66, 14))
                .build();

        Action readyClawTick1 = roadrunnerDrive.actionBuilder(new Pose2d(66, 14, Math.toRadians(0))) //get in position to grab first tick
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(41, -65, Math.toRadians(-29)), Math.toRadians(-90))
                .build();
                //deploy intake after this + grab piece

        Action clawTick1 = roadrunnerDrive.actionBuilder(new Pose2d(41, -65, Math.toRadians(-29))) //new Pose2d(41, -62, Math.toRadians(-32))
                .turnTo(Math.toRadians(-152))
                .build(); //let go of piece after this (deploy intake) + stowIntake

        Action readyClawTick2 = roadrunnerDrive.actionBuilder(new Pose2d(41, -62, Math.toRadians(-152)))
                .strafeToLinearHeading(new Vector2d(46, -82.8), Math.toRadians(-27))
                //.turnTo(Math.toRadians(-33.19))
                .build(); //deploy intake + grab piece after this

        Action clawTick2 = roadrunnerDrive.actionBuilder(new Pose2d(46, -82.8, Math.toRadians(-27)))
                .turnTo(Math.toRadians(-159))
                .build(); //let go of piece after this (deploy intake) + (stowIntake + armIntakePose, ready2ndSpec)

        Action readySecondSpec = roadrunnerDrive.actionBuilder(new Pose2d(46, -82.8, Math.toRadians(-159)))
                .strafeToLinearHeading(new Vector2d(70, -94.5), Math.toRadians(0))
                .strafeTo(new Vector2d(20.6, -102))
                .build(); //simultaneously stowIntake, armIntakePose

        Action pushTicks = roadrunnerDrive.actionBuilder(new Pose2d(68, 14, Math.toRadians(0))) //put arm in spec intake pose while running this
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -38), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(100, -41), Math.toRadians(0)) //96, -42
                .splineToConstantHeading(new Vector2d(113, -50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(92, -79), Math.toRadians(-180))
                .strafeTo(new Vector2d(27, -78))
                .strafeTo(new Vector2d(91, -79))
                .waitSeconds(0.8)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(113, -85), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(90, -102), Math.toRadians(-180))
                .strafeTo(new Vector2d(27, -102))
                .build();

        Action secondSpec = roadrunnerDrive.actionBuilder(new Pose2d(20.6, -102, Math.toRadians(0)))
                .waitSeconds(1)
                /*
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(70.5, 13.5), Math.toRadians(0))

                 */
                .strafeTo(new Vector2d(25, -45))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(72, 16.5), Math.toRadians(0))
                .build();

        Action thirdSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(66, 16.5, Math.toRadians(0)))
                /*
                .setTangent(-180)
                .splineToConstantHeading(new Vector2d(5.45, -87), Math.toRadians(-180))

                 */

                .setTangent(180)
                .splineToConstantHeading(new Vector2d(30, -33), Math.toRadians(270))
                .strafeTo(new Vector2d(22, -75))
                .build();


        Action thirdSpec = roadrunnerDrive.actionBuilder(new Pose2d(22, -75, Math.toRadians(0)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(25, -45))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(72, 21.5), Math.toRadians(0))
                .build();

        Action park = roadrunnerDrive.actionBuilder(new Pose2d(66, 21.5, Math.toRadians(0)))
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(15, -70), Math.toRadians(270))
                .build();


//        Action spike1 = roadrunnerDrive.actionBuilder(new Pose2d(68, 13, Math.toRadians(0)))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(41 , -44, Math.toRadians(-40)), Math.toRadians(-90))
//                .build();
//
//        Action zone1 = roadrunnerDrive.actionBuilder(new Pose2d(41, -44, Math.toRadians(-40)))
//                .turnTo(Math.toRadians(-155))
//                .build();

        Action waitAction = roadrunnerDrive.actionBuilder(new Pose2d(66, 14.5, Math.toRadians(0)))
            .waitSeconds(0.7)
            .build();




        //CALL ACTIONS HERE


        Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            arm.armUp(),
                            preloadSpec
                    ),
                    arm.armS()//,

                    //intake.intakeStow()
                )
        );

        specClaw.setPosition(0); //open claw

        Actions.runBlocking(new ParallelAction(
                intake.intakeStow(),
                readyClawTick1
        ));

        Actions.runBlocking(
                intake.intakeDeploy()
        );

        sleep(400);

        Actions.runBlocking(
                intake.intakeGrab()
        );

        sleep(400);

        Actions.runBlocking(new SequentialAction(
                clawTick1,
                intake.intakeDeploy()
        ));

        sleep(300);

        Actions.runBlocking(
                intake.intakeStow()
        );

        sleep(400);

        Actions.runBlocking(new SequentialAction(
                readyClawTick2,
                intake.intakeDeploy()
        ));

        sleep(400);

        Actions.runBlocking(
                intake.intakeGrab()
        );

        sleep(400);

        Actions.runBlocking(new SequentialAction(
                clawTick2,
                intake.intakeDeploy()
        ));

        sleep(300);

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        intake.intakeStow(),
                        arm.armSpecIntake()
                ),
                readySecondSpec
        ));

        //sleep(10000);

        /*

        Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            arm.armUp(),
                            preloadSpec
                    ),
                    arm.armS()//,

                    //intake.intakeStow()
                )
        );

         specClaw.setPosition(0); //open claw

        Actions.runBlocking(
                new ParallelAction(
                        pushTicks,
                        arm.armSpecIntake()
                )
        );
        */



        specClaw.setPosition(0.23);
        sleep(300);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction (
                        arm.armReady(),
                        secondSpec
                ),
                arm.armScore())
        );

        specClaw.setPosition(0);
        sleep(200);

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        waitAction,
                        arm.armSpecIntake()
                ),
                thirdSpecPickup
        ));

        specClaw.setPosition(0.23);
        sleep(400);

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        arm.armReady(),
                        thirdSpec
                ),
                arm.armScore()
        ));

        specClaw.setPosition(0);
        sleep(200);

        Actions.runBlocking(new SequentialAction(
                park,
                arm.armStow()
        ));



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