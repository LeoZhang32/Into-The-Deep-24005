package org.firstinspires.ftc.teamcode.itd;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class test_1107_short extends LinearOpMode {


    Servo intakeRight;
    //    Boolean intakeRight_button_pressed = false;
    Boolean isA1 = true;

    Servo intakeLeft;
//    Boolean intakeLeft_button_pressed = false;
//    Boolean intakeLeft_extended = false;

    Servo intakeBack;
    //    Boolean intakeBack_button_pressed = false;
    Boolean isB2 = false;


    @Override
    public void runOpMode() throws InterruptedException {

        intakeRight = hardwareMap.servo.get("intakeRight");
        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeBack = hardwareMap.servo.get("intakeBack");

        // Set the initial positions for intakeRight, intakeLeft and intakeBack
        intakeRight.setPosition(0.43); // A1 position
        intakeLeft.setPosition(0.57); // A1 position
        intakeBack.setPosition(0.6); // B1 position


        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {


            if (gamepad1.x) {
                isA1 = !isA1;

                if (isA1) {
                    intakeRight.setPosition(0.43);// Move Servo intakeRight to A1
                    intakeLeft.setPosition(0.57);// Move Servo intakeLeft to A1
                    intakeBack.setPosition(0.6);// Move Servo intakeBack to B1
                } else {
                    intakeRight.setPosition(0.16);// Move Servo intakeRight to A2
                    intakeLeft.setPosition(0.84);// Move Servo intakeLeft to A2

                    // Toggle intakeBack position between B2 and B3
                    isB2 = !isB2;
                    if (isB2) {
                        intakeBack.setPosition(0.9); // Move intakeBack to B2
                    } else {
                        intakeBack.setPosition(0.96); // Move intakeBack to B3
                    }
                }
                // Small delay to prevent rapid toggling
                sleep(200);

            }
        }
    }
}