package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="PID Test1")
public class teleop_pidtest1 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(145, shooterTop.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("velocity top", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("reference", 145);
            dashboardTelemetry.update();
            shooterTop.setPower(power);
            shooterBottom.setPower(power);
            telemetry.update();
        }
    }
    public double PIDControl (double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}

