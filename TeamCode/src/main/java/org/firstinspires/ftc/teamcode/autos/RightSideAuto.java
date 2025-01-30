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
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeWrist.setPosition(1);
                //MAKE SURE TO MOVE PIVOT ARM UP BEFORE MOVING LINKAGES!!!
                linkage1.setPosition(0.2);
                linkage2.setPosition(0.2);
                intakeCR.setPower(-1);

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
                linkage1.setPosition(0.2);
                linkage2.setPosition(0.2);
                intakeCR.setPower(1);

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
                intakeCR.setPower(0);

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
                specWrist.setPosition(0.07);

                r = r*r + 1;

                return armMotor.isBusy();

            }
        }

        public Action armSpecIntake() {
            return new ArmSpecIntake();
        }

        public class ArmScore implements Action { //move arm to score
            private boolean initialized = false;
            private int r = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specWrist.setPosition(0.7);

                if (!initialized && r > 200) {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(-720);
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

    }




    private ElapsedTime runtime = new ElapsedTime();

    private Servo claw;



    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class, "claw");

        MecanumDrive roadrunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        //INITIALIZE CLASSES HERE!!!!!
        //DO PRE STUFF HERE (MOVE SERVOS, ETC. IF APPLICABLE!)

        waitForStart();

        //START BUILDING ACTIONS HERE; AND CALL ACTIONS AFTER!!


    }


}