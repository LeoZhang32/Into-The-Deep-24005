package org.firstinspires.ftc.teamcode.itd.tests;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Config
@TeleOp
public class slides_pidf_test extends LinearOpMode {

    DcMotorEx motor;

    ElapsedTime pidfTimer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double p = 0, i = 0, d = 0;
    public static double targetPosition = 3000;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int targetPosition = 3000;
        while (opModeIsActive()){
            double power = returnPower(targetPosition, motor.getCurrentPosition());
            packet.put("power", power);
            packet.put("position", motor.getCurrentPosition());
            packet.put("error", lastError);
            motor.setPower(power);

            dashboard.sendTelemetryPacket(packet);

        }
    }
    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * pidfTimer.seconds();
        double derivative = (error - lastError) / pidfTimer.seconds();
        lastError = error;

        pidfTimer.reset();

        double output = (error * p) + (derivative * d) + (integralSum * i);
        return output;
    }
}
