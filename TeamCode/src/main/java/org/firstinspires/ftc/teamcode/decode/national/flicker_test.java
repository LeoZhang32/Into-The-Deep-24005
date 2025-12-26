package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

    @Override
    public void runOpMode() throws InterruptedException {

        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");

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
