package org.firstinspires.ftc.teamcode.decode.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class flicker_test extends LinearOpMode {

    Servo flicker1;
    boolean ball1_button_pressed;
    boolean ball1_released;
    Servo flicker2;
    boolean ball2_button_pressed;
    boolean ball2_released;
    Servo flicker3;
    boolean ball3_button_pressed;
    boolean ball3_released;
    public static double home1 = 0.65;
    public static double home2 = 0.85;
    public static double home3 = 0.76;
    public static double score1 = 0.95;
    public static double score2 = 0.55;
    public static double score3 = 0.46;

    @Override
    public void runOpMode() throws InterruptedException {

        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

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


        }


    }

    public void updateBooleans() {
        if (ball1_released) {
            flicker1.setPosition(score1);
        } else {
            flicker1.setPosition(home1);// lower position
        }

        if (ball2_released) {
            flicker2.setPosition(score2);
        } else {
            flicker2.setPosition(home2);// lower position
        }

        if (ball3_released) {
            flicker3.setPosition(score3);
        } else {
            flicker3.setPosition(home3);// lower position
        }

    }
}
