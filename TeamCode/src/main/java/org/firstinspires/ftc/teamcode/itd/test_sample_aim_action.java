package org.firstinspires.ftc.teamcode.itd;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class test_sample_aim_action extends LinearOpMode {


    Servo intakeRight;
    //    Boolean intakeRight_button_pressed = false;
    Boolean isA1 = true;

    Servo intakeLeft;
//    Boolean intakeLeft_button_pressed = false;
//    Boolean intakeLeft_extended = false;

    Servo intakeBack;
    //    Boolean intakeBack_button_pressed = false;
    Boolean isB2 = false;
    int pressCount = 0;

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

            if (gamepad1.x) { // Prevent "button held down" behavior
                // Increment the press count and ensure it loops between 0 and 2
                pressCount++;


                // Cycle through the positions
                if (pressCount == 1) {
                    intakeRight.setPosition(0.16);
                    intakeLeft.setPosition(0.84);
                    intakeBack.setPosition(0.9);
                    // Position 1 (middle position)
                } else if (pressCount == 2) {
                    intakeRight.setPosition(0.16);
                    intakeLeft.setPosition(0.84);
                    intakeBack.setPosition(0.96); // Position 2 (taking sample)
                } else if (pressCount == 3) {
                    intakeRight.setPosition(0.43); //this is the initial position
                    intakeLeft.setPosition(0.57); //this = 1-intakeRight position
                    intakeBack.setPosition(0.6); //this is the initial position
                    // Position 3 (drop sample into bucket)
                    pressCount = 0; // Reset to cycle back to initial position
                }

                sleep(200);
            }

            if (gamepad1.y) {
                pressCount = 1;

                if (pressCount == 1){
                    intakeRight.setPosition(0.16);
                    intakeLeft.setPosition(0.84);
                    intakeBack.setPosition(0.9);
                }

                sleep(200);
            }

            if (gamepad1.b) {
                pressCount = 3;

                if (pressCount == 3){
                    intakeRight.setPosition(0.43); //this is the initial position
                    intakeLeft.setPosition(0.57); //this = 1-intakeRight position
                    intakeBack.setPosition(0.6); //this is the initial position
                    // Position 3 (drop sample into bucket)
                    pressCount = 0; // Reset to cycle back to initial position
                }

                sleep(200);
            }


//            if (gamepad1.x) {
//                isA1 = !isA1;
//
//                if (isA1) {
//                    intakeRight.setPosition(0.43);// Move Servo intakeRight to A1
//                    intakeLeft.setPosition(0.57);// Move Servo intakeLeft to A1
//                    intakeBack.setPosition(0.6);// Move Servo intakeBack to B1
//                } else {
//                    intakeRight.setPosition(0.16);// Move Servo intakeRight to A2
//                    intakeLeft.setPosition(0.84);// Move Servo intakeLeft to A2
//
//                    // Toggle intakeBack position between B2 and B3
//                    isB2 = !isB2;
//                    if (isB2) {
//                        intakeBack.setPosition(0.9); // Move intakeBack to B2
//                    } else {
//                        intakeBack.setPosition(0.96); // Move intakeBack to B3
//                    }
//                }
//                // Small delay to prevent rapid toggling
//                sleep(200);
//
//
        }
    }
}

