package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;
import org.firstinspires.ftc.teamcode.decode.DecodeRobotHardware;

@Config
@TeleOp(name="intake + flicker + PID Test1")
public class intake_flicker_pid_test1 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    DcMotorEx shooterTop;
    DcMotorEx shooterBottom;

    DcMotorEx intake;

    double integralSum = 0;
    double Kp = 0.0325;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0.0038;
    double targetVelocity = 190;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    Servo flicker1;
    boolean ball1_button_pressed;
    boolean ball1_released;
    Servo flicker2;
    boolean ball2_button_pressed;
    boolean ball2_released;
    Servo flicker3;
    boolean ball3_button_pressed;
    boolean ball3_released;

    @Override
    public void runOpMode() throws InterruptedException {

        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
        shooterBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        CycleGamepad cyclegamepad1 = new CycleGamepad(gamepad1);


        waitForStart();
        while (opModeIsActive()) {
            cyclegamepad1.updateRB(2);
            double power = PIDControl(targetVelocity, shooterTop.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("velocity top", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("velocity", shooterTop.getVelocity(AngleUnit.DEGREES));
            dashboardTelemetry.addData("reference", targetVelocity);
            dashboardTelemetry.update();
            if (cyclegamepad1.rbPressCount == 0) {
                shooterTop.setPower(0);
                shooterBottom.setPower(0);
            } else {
                shooterTop.setPower(power);
                shooterBottom.setPower(power);
            }

            telemetry.update();

            if (gamepad1.x) {
                if (!ball1_button_pressed) {
                    ball1_released = !ball1_released;
                }
                ball1_button_pressed = true;

            } else ball1_button_pressed = false;
            updateBooleans();


            if (gamepad1.a) {
                if (!ball2_button_pressed) {
                    ball2_released = !ball2_released;
                }
                ball2_button_pressed = true;

            } else ball2_button_pressed = false;
            updateBooleans();


            if (gamepad1.b) {
                if (!ball3_button_pressed) {
                    ball3_released = !ball3_released;
                }
                ball3_button_pressed = true;

            } else ball3_button_pressed = false;
            updateBooleans();



            if (gamepad1.dpad_up){
                intake.setPower(1);
            }
            else if (gamepad1.dpad_down){
                intake.setPower(-1);
            }
            else intake.setPower(0);



        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

    public void updateBooleans() {
        if (ball1_released) {
            flicker1.setPosition(0.95);
        } else {
            flicker1.setPosition(0.65);// lower position
        }

        if (ball2_released) {
            flicker2.setPosition(0.55);
        } else {
            flicker2.setPosition(0.85);// lower position
        }

        if (ball3_released) {
            flicker3.setPosition(0.46);
        } else {
            flicker3.setPosition(0.76);// lower position
        }

    }
}
