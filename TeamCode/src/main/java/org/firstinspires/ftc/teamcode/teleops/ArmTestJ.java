package org.firstinspires.ftc.teamcode.teleops;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Arm Test J")
public class ArmTestJ extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    View relativeLayout;

    DcMotorEx armMotor = null;
    public static double armPower = 0.5;
    public static int armPose = 0;

    int armTargetPos = -275;

    public static double kP = 5; //10
    public static double kI = 0.05; //0.05
    public static double kD = 0; //0
    public static double kF = 0;

    public double thetaInit = Math.toRadians(-38.1176);
    public double theta = thetaInit;

    PIDFCoefficients pidfCoeff = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init(){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        UpdateTelemetry();
    }

    @Override
    public void init_loop(){

        UpdateTelemetry();

    }

    @Override
    public void start(){

        pidfCoeff = new PIDFCoefficients(kP, kI, kD, kF);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeff);
        armMotor.setTargetPosition(armTargetPos);
        armMotor.setPower(armPower);

        UpdateTelemetry();

    }

    @Override
    public void loop(){
        armMotor.setTargetPosition(armTargetPos);

        theta = thetaInit + (-1.0 * Math.PI * armMotor.getCurrentPosition() / 255);
//        armMotor.setPower(0.3);
        UpdateTelemetry();
    }

    @Override
    public void stop(){

    }

    public void UpdateTelemetry(){
        telemetry.addData("arm motor clicks", armMotor.getCurrentPosition());
        telemetry.addData("arm motor velocity", armMotor.getVelocity());
        telemetry.addData("arm motor power", armMotor.getPower());
        telemetry.addData("arm motor target pos", armMotor.getTargetPosition());
        telemetry.addData("PIDF coef", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("arm motor theta", theta);

        packet.put("arm motor clicks", armMotor.getCurrentPosition());
        packet.put("arm motor velocity", armMotor.getVelocity());
        packet.put("arm motor power", armMotor.getPower());
        packet.put("arm motor target pos", armMotor.getTargetPosition());
        packet.put("PIDF coef", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        packet.put("arm motor theta", theta);

        dashboard.sendTelemetryPacket(packet);
    }
}
