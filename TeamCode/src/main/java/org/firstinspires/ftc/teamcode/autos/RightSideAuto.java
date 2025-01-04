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
import org.firstinspires.ftc.teamcode.teleops.Constants;

import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="right side auto", group="beta")
public class RightSideAuto extends LinearOpMode {

    public class Wrist {
        private Servo wristS;

        public Wrist(HardwareMap hardwareMap) {
            wristS = hardwareMap.get(Servo.class, "wrist");

        }

        public class WristIntakeSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristS.setPosition(1 - 0.6631);
                return false;
            }
        }

        public Action wristIntakeSpecimen() {
            return new WristIntakeSpecimen();
        }

        public class WristReadySpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristS.setPosition(1 - 0.5);
                return false;
            }
        }

        public Action wristReadySpecimen() {
            return new WristReadySpecimen();
        }

    }



    public class Claw {
        private Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.5);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Slide {
        private DcMotor slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, "slide");
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setDirection(DcMotor.Direction.REVERSE);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class SlideIn implements Action {
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(1);
                    slide.setTargetPosition((int) Constants.slide_retracted_pose);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;
                return slide.isBusy();
            }
        }

        public Action slideIn() {
            return new SlideIn();
        }

        public class SlideOut implements Action {
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(1);
                    slide.setTargetPosition((int) Constants.slide_specimen_high_rung);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;
                return slide.isBusy();
            }
        }

        public Action slideOut() {
            return new SlideOut();
        }

    }

    public class Pivot {
        private DcMotorEx pivot;

        public Pivot(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(DcMotorEx.class, "pivot");
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setDirection(DcMotor.Direction.REVERSE);
            pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class PivotUp implements Action {
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot.setPower(1);
                    pivot.setTargetPosition((int) Constants.pivot_high_pose);
                    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r=r*r + 1;
                return pivot.isBusy();
            }
        }

        public Action pivotUp() {
            return new PivotUp();
        }

        public class PivotUp2 implements Action {
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot.setPower(1);
                    pivot.setTargetPosition((int) Constants.pivot_high_pose_2);
                    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r=r*r + 1;
                return pivot.isBusy();
            }
        }

        public Action pivotUp2() {
            return new PivotUp2();
        }

        public class PivotDown implements Action {
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot.setPower(1);
                    pivot.setTargetPosition((int) Constants.pivot_intake_pose);
                    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }

                r = r*r + 1;

                return pivot.isBusy();
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

    }


    private ElapsedTime runtime = new ElapsedTime();

    //private DcMotor slide = null;
    //private DcMotorEx pivot = null;

    //private Servo wrist = null;

    //double servoSetpoint = 0;

    //private Servo clawServo = null;

    //private boolean boolPivotUp = false;



    @Override
    public void runOpMode() throws InterruptedException {
        //slide = hardwareMap.get(DcMotor.class, "slide");
        //pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        //wrist = hardwareMap.get(Servo.class, "wrist");

        //clawServo = hardwareMap.get(Servo.class, "claw");

        MecanumDrive roadrunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Slide slide = new Slide(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        Action PreloadSpec = roadrunnerDrive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(63.35, 0)) //preload
                .build();

        Action PushTicks = roadrunnerDrive.actionBuilder(new Pose2d(63.35, 0, Math.toRadians(0)))
                .setTangent(Math.toRadians(180)) //pushticks

                /* test
                .splineToSplineHeading(new Pose2d(50, -17, Math.toRadians(90)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(106, -57, Math.toRadians(180)), Math.toRadians(0))
                .setReversed(true) //TESTING!
                .splineToConstantHeading(new Vector2d(110, -67.5), Math.toRadians(270))

                 */

                .splineToConstantHeading(new Vector2d(35, -30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(107, -58), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(108, -68), Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(105, -73), Math.toRadians(180))


                //.splineToSplineHeading(new Pose2d(30, -17, Math.toRadians(90)), Math.toRadians(270))
                //.splineToSplineHeading(new Pose2d(102, -57, Math.toRadians(180)), Math.toRadians(0))

                //.splineTo(new Vector2d(30, -17), Math.toRadians(270)) #2

                //.splineTo(new Vector2d(57, -56), Math.toRadians(0))
                //.setTangent(Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(102, -57), Math.toRadians(0))

                //.splineTo(new Vector2d(102, -57), Math.toRadians(0)) #2

                //.setReversed(true)



                //.setTangent(Math.toRadians(0))

                //.splineToConstantHeading(new Vector2d(105, -64), Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(102, -71), Math.toRadians(180))
                //.strafeTo(new Vector2d(30, -71))

                .splineToConstantHeading(new Vector2d(25, -68), Math.toRadians(180))


                .setTangent(Math.toRadians(0))

                .lineToXSplineHeading(105, Math.toRadians(180))

                //.splineToConstantHeading(new Vector2d(106, -71), Math.toRadians(0)) test


                //.lineToXSplineHeading(106, Math.toRadians(180))


                .setReversed(true)

                .setTangent(270)
                .splineToConstantHeading(new Vector2d(102, -91), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -94), Math.toRadians(180))

                .setTangent(0)

                .splineToConstantHeading(new Vector2d(101, -96), Math.toRadians(0))//.splineToConstantHeading(new Vector2d(108, -87), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(102.4, -100), Math.toRadians(270))

                //.splineToConstantHeading(new Vector2d(103.3, -93), Math.toRadians(180))
                //.splineToConstantHeading(new Vector2d(102, -110), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30, -100.5), Math.toRadians(180))

                .setTangent(Math.toRadians(90))

//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(108, -95), Math.toRadians(0))
//
//                .splineToConstantHeading(new Vector2d(103.3, -98), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(30, -103), Math.toRadians(180))
//
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(108, -103), Math.toRadians(0))
//
//                .splineToConstantHeading(new Vector2d(103.3, -108), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(30, -103), Math.toRadians(180))

                //.splineToConstantHeading(new Vector2d(7.414, -100), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -65), Math.toRadians(180))

                .build();

        //.setTangent(Math.toRadians(0))
        //.splineToConstantHeading(new Vector2d(105, -80), Math.toRadians(-90))
        //.splineToConstantHeading(new Vector2d(102, -90), Math.toRadians(180))
        //.strafeTo(new Vector2d(30, -90))
        //.setTangent(Math.toRadians(0))
        //.splineToConstantHeading(new Vector2d(102, -90), Math.toRadians(0))
        //.setTangent(Math.toRadians(0))
        //.splineToConstantHeading(new Vector2d(105, -99), Math.toRadians(-90))
        //.splineToConstantHeading(new Vector2d(102, -108), Math.toRadians(180))
        //.splineToConstantHeading(new Vector2d(30, -103), Math.toRadians(180))
        //.splineToConstantHeading(new Vector2d(6.3, -85), Math.toRadians(180))
        //.build();



                /*
                .strafeTo(new Vector2d(45, 0))
                .splineToSplineHeading(new Pose2d(102, -57, Math.toRadians(180)), Math.toRadians(0))
                .setReversed(true)
                .strafeTo(new Vector2d(105, -71))
                .strafeTo(new Vector2d(30, -71))
                .strafeTo(new Vector2d(105, -71))
                .strafeTo(new Vector2d(105, -90))
                .strafeTo(new Vector2d(30, -90))
                .strafeTo(new Vector2d(105, -90))
                .strafeTo(new Vector2d(105, -108))
                .strafeTo(new Vector2d(6.3, -100))
                //.strafeTo(new Vector2d(30, -108))
                //.strafeTo(new Vector2d(6.3, -85))
                .build();

                 */

        Action SecondSpec = roadrunnerDrive.actionBuilder(new Pose2d(6, -65, Math.toRadians(180)))//keep at 7.38 instead of 6 - works for some reason //new Pose2d(6.3, -85, Math.toRadians(180)))
                .setReversed(true) //second spec
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(20, -50), Math.toRadians(90))
                .lineToYSplineHeading(1, Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(63.35, 8), Math.toRadians(0))
                .build();

                /*
                .setReversed(true)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(61, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
                 */

        Action ThirdSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(63.35, 8, Math.toRadians(0)))
                .setReversed(false) //third spec pickup
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(20, -2), Math.toRadians(-90))
                .lineToYSplineHeading(-58, Math.toRadians(180))
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(10, -65), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(7.3, -65), Math.toRadians(180))
                .build();

                /*
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(6.3, -85, Math.toRadians(180)), Math.toRadians(180))
                .build();
                 */

        Action ThirdSpec = roadrunnerDrive.actionBuilder(new Pose2d(7.3, -65, Math.toRadians(180)))
                .setReversed(true) //third spec
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(20, -50), Math.toRadians(90))
                .lineToYSplineHeading(3, Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(63.7, 10.5), Math.toRadians(0))
                .build();

                /*
                .setReversed(true) //third spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(63.35, 1, Math.toRadians(0)), Math.toRadians(0))
                .build();

                 */

        Action Park = roadrunnerDrive.actionBuilder(new Pose2d(63.35, 10.5, Math.toRadians(0)))
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(10, -100), Math.toRadians(270))
                .build();


        /*FIX EVERYTHING BELOW
        Action FourthSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(63.35, 6.5, Math.toRadians(0)))
                .setReversed(false) //fourth spec pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(7.414, -90, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Action FourthSpec = roadrunnerDrive.actionBuilder(new Pose2d(7.414, -90, Math.toRadians(180)))
                .setReversed(true) //fourth spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(63.35, 2, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action FifthSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(63.35, 2, Math.toRadians(0)))
                .setReversed(false) //fifth spec pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(7.414, -90, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Action FifthSpec = roadrunnerDrive.actionBuilder(new Pose2d(7.414, -90, Math.toRadians(180)))
                .setReversed(true) //fifth spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(63.35, 3, Math.toRadians(0)), Math.toRadians(0))
                .build();


         */ //FIX EVERYTHING ABOVE



        waitForStart();

        Actions.runBlocking(claw.closeClaw());

        //testing trajectories for now; when done, add subsystems through parallel + sequential complex actions
        Actions.runBlocking(new ParallelAction(
                PreloadSpec,
                new SequentialAction(
                        pivot.pivotUp(),
                        wrist.wristReadySpecimen()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                slide.slideOut(),
                claw.openClaw(),
                slide.slideIn()
        ));
//
//        Actions.runBlocking(new SequentialAction(
//                slide.slideOut(),
//                claw.openClaw(),
//                slide.slideIn()
//        ));

        Actions.runBlocking(new ParallelAction(
                PushTicks,
                new SequentialAction(
                        pivot.pivotDown(),
                        wrist.wristIntakeSpecimen()
                )
        ));

        sleep(50);

        Actions.runBlocking(claw.closeClaw());

        sleep(200);

        Actions.runBlocking(new SequentialAction(
                pivot.pivotUp(),
                wrist.wristReadySpecimen(),
                SecondSpec,
                slide.slideOut(),
                claw.openClaw(),
                slide.slideIn()
        ));


        Actions.runBlocking(new ParallelAction(
                ThirdSpecPickup,
                new SequentialAction(
                        pivot.pivotDown(),
                        wrist.wristIntakeSpecimen()
                )
        ));

        sleep(50);

        Actions.runBlocking(claw.closeClaw());

        sleep(200);

        Actions.runBlocking(new SequentialAction(
                pivot.pivotUp2(),
                wrist.wristReadySpecimen(),
                ThirdSpec
        ));

        Actions.runBlocking(new SequentialAction(
                slide.slideOut(),
                claw.openClaw(),
                slide.slideIn(),
                new ParallelAction(
                        pivot.pivotDown(),
                        Park
                )
        ));

        /*
        sleep(500);
        Actions.runBlocking(ThirdSpec);
        sleep(500);
        Actions.runBlocking(FourthSpecPickup);
        sleep(500);
        Actions.runBlocking(FourthSpec);
        sleep(500);

         */

        /* ADD 5 SPEC AFTER MEET 3!
        Actions.runBlocking(FifthSpecPickup);
        sleep(500);
        Actions.runBlocking(FifthSpec);

         */





        /*
        Action PreloadSpec = roadrunnerDrive.actionBuilder(new Pose2d(9, -63, Math.toRadians(90)))
                .lineToY(-15)
                .build();

        Action PushTicks = roadrunnerDrive.actionBuilder(new Pose2d(9, -15, Math.toRadians(90)))
                .lineToYSplineHeading(-15, Math.toRadians(-90))
                .setReversed(true)
                .strafeTo(new Vector2d(9, -20))
                .strafeTo(new Vector2d(63, -20))
                .strafeTo(new Vector2d(63, 43))
                .strafeTo(new Vector2d(83, 43))
                .strafeTo(new Vector2d(83, -38))
                .strafeTo(new Vector2d(83, 43))
                .strafeTo(new Vector2d(103, 43))
                .strafeTo(new Vector2d(103, -38))
                .strafeTo(new Vector2d(103, 43))
                .strafeTo(new Vector2d(117, 43))
                .strafeTo(new Vector2d(117, -38))
                .strafeTo(new Vector2d(90, -38))
                .strafeTo(new Vector2d(90, -58.29))
                .build();

       Action SecondSpec = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.29, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0, -3, Math.toRadians(90)), Math.toRadians(-90))//try 90 as tangent too
                .build();

        Action ThirdSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(0, -3, Math.toRadians(90)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(90, -58.4, Math.toRadians(-90)), Math.toRadians(90))//try -90 as tangent too
                .build();

        Action ThirdSpec = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.4, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-6, -3, Math.toRadians(90)), Math.toRadians(-90))//try 90 as tangent too
                .build();

        Action FourthSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(-6, -3, Math.toRadians(90)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(90, -58.4, Math.toRadians(-90)), Math.toRadians(90)) //try -90 as tangent too
                .build();

        Action FourthSpec = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.4, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-11, -3, Math.toRadians(90)), Math.toRadians(-90)) //try 90 as tangent too
                .build();

        Action FifthSpecPickup = roadrunnerDrive.actionBuilder(new Pose2d(-11, -3, Math.toRadians(90)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(90, -58.4, Math.toRadians(-90)), Math.toRadians(90))//try -90 as tangent too
                .build();

        Action FifthSpec = roadrunnerDrive.actionBuilder(new Pose2d(90, -58.4, Math.toRadians(-90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-16, -3, Math.toRadians(90)), Math.toRadians(-90))//try 90 as tangent too
                .build();

        waitForStart();

        runtime.reset();

        //MOVEMENT STARTS HERE
        wrist.setPosition(0);

        Actions.runBlocking(claw.closeClaw());

        Actions.runBlocking(new SequentialAction(//ParallelAction(
                PreloadSpec,
                //new SequentialAction(
                        pivot.pivotUpOne()//,
                        //wrist1.wristScoreSpecimen1(),
                //)
        ));

        wristScoreSpecimen1();

        Actions.runBlocking(new SequentialAction(
                slide.slideOutOne(),
                claw.openClaw()
        ));

        sleep(230);

        Actions.runBlocking(new SequentialAction(
                slide.slideIn(),
                pivot.pivotDown()
        ));

        Actions.runBlocking(new SequentialAction(
                PushTicks,
                claw.closeClaw()
        ));


        Actions.runBlocking(new SequentialAction(//ParallelAction(
                SecondSpec,
                //new SequentialAction(
                        pivot.pivotUpTwo()//,
                        //wrist1.wristScoreSpecimen2()
                //)
        ));


        wristScoreSpecimen2();

        Actions.runBlocking(new SequentialAction(
                slide.slideOutOne(),
                claw.openClaw()
        ));

        sleep(230);

        Actions.runBlocking(new SequentialAction(
                slide.slideIn(),
                pivot.pivotDown()
        ));

        wristScoreSpecimen1();

        Actions.runBlocking(new SequentialAction(
                //new ParallelAction(
                        ThirdSpecPickup,
                        //wrist1.wristScoreSpecimen1()),//works as intake pose
                claw.closeClaw()
        ));


        Actions.runBlocking(new SequentialAction(//ParallelAction(
                ThirdSpec,
                //new SequentialAction(
                        pivot.pivotUpTwo()//,
                        //wrist1.wristScoreSpecimen2()
                //)
        ));

        wristScoreSpecimen2();

        Actions.runBlocking(new SequentialAction(
                slide.slideOutOne(),
                claw.openClaw()
        ));

        sleep(230);

        Actions.runBlocking(new SequentialAction(
                slide.slideIn(),
                pivot.pivotDown()
        ));

        wristScoreSpecimen1();

        Actions.runBlocking(new SequentialAction(
                //new ParallelAction(
                        FourthSpecPickup,
                        //wrist1.wristScoreSpecimen1()//works as intake pose
                //),
                claw.closeClaw()
        ));



        Actions.runBlocking(new SequentialAction(//ParallelAction(
                FourthSpec,
                //new SequentialAction(
                        pivot.pivotUpTwo()//,
                        //.wristScoreSpecimen2()
                //)
        ));

        wristScoreSpecimen2();

        Actions.runBlocking(new SequentialAction(
                slide.slideOutOne(),
                claw.openClaw()
        ));

        sleep(230);

        Actions.runBlocking(new SequentialAction(
                slide.slideIn(),
                pivot.pivotDown()
        ));

        wristScoreSpecimen1();

        Actions.runBlocking(new SequentialAction(
                //new ParallelAction(
                        FifthSpecPickup,
                        //wrist1.wristScoreSpecimen1()//works as intake pose
                //),
                claw.closeClaw()
        ));


        Actions.runBlocking(new SequentialAction(//ParallelAction(
                FifthSpec,
                //new SequentialAction(
                        pivot.pivotUpTwo()//,
                        //wrist1.wristScoreSpecimen2()
                //)
        ));

        wristScoreSpecimen2();

        Actions.runBlocking(new SequentialAction(
                slide.slideOutOne(),
                claw.openClaw()
        ));

        sleep(230);

        Actions.runBlocking(new SequentialAction(
                slide.slideIn(),
                pivot.pivotDown()
        ));

        */
    }

        /*
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

                //.strafeTo(new Vector2d(110, -60))
                //.splineTo(new Vector2d(100, -50), Math.toRadians(90))
                //.splineTo(new Vector2d(90, -60), Math.toRadians(180))


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

        */


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


    //}


    /*
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


    }

     */

    /*
    public void slideOutTwo() {
        //slide.setPower(-gamepad2.right_stick_y * 1.5);

        slide.setTargetPosition((int) Constants.slide_specimen_high_rung_2);
        slide.setPower(1);
        int r = 0;
        while (slide.isBusy()) {
            r = r * r + 1;
        }

    }

     */

    //public void openClaw() {
    //clawServo.setPosition(0.5);
    //}

    //public void closeClaw() {
    //clawServo.setPosition(1);
    //}



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