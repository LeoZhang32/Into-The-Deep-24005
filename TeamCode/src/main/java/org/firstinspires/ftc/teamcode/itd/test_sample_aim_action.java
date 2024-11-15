package org.firstinspires.ftc.teamcode.itd;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class test_sample_aim_action extends LinearOpMode {


    Servo intakeRight;
    Servo intakeLeft;
    Servo intakeBack;
    int pressCount = 0;

    Boolean previousXState = false;
    Boolean currentXState;
    Boolean previousYState = false;
    Boolean currentYState;
    Boolean previousBState = false;
    Boolean currentBState;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeRight = hardwareMap.servo.get("intakeRight");
        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeBack = hardwareMap.servo.get("intakeBack");

        // Set the initial positions for intakeRight, intakeLeft and intakeBack
        intakeRight.setPosition(0.6);
        intakeLeft.setPosition(0.4);
        intakeBack.setPosition(0.5);


        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            currentXState = gamepad1.x;
            currentYState = gamepad1.y;
            currentBState = gamepad1.b;

            if (currentXState && !previousXState) { // Prevent "button held down" behavior
                // Increment the press count and ensure it loops between 0 and 2
                pressCount++;

                // Cycle through the positions
                if (pressCount == 1) {
                    intakeRight.setPosition(0.3);
                    intakeLeft.setPosition(0.7);
                    intakeBack.setPosition(0.73);
                    // Position 1 (aiming position)
                } else if (pressCount == 2) {
                    intakeRight.setPosition(0.3);
                    intakeLeft.setPosition(0.7);
                    intakeBack.setPosition(0.8); // Position 2 (taking sample)
                } else if (pressCount == 3) {
                    intakeRight.setPosition(0.6); //this is the initial position
                    intakeLeft.setPosition(0.4); //this = 1-intakeRight position
                    intakeBack.setPosition(0.5); //this is the initial position
                    // Position 3 (drop sample into bucket)
                    pressCount = 0; // Reset to cycle back to initial position
                }
            }

            previousXState = currentXState;

            if (currentYState && !previousYState) {
                pressCount = 1;

                if (pressCount == 1){
                    intakeRight.setPosition(0.3);
                    intakeLeft.setPosition(0.7);
                    intakeBack.setPosition(0.73);
                }
            }

            previousYState = currentYState;

            if (currentBState && !previousBState) {
                pressCount = 3;

                if (pressCount == 3){
                    intakeRight.setPosition(0.6); //this is the initial position
                    intakeLeft.setPosition(0.4); //this = 1-intakeRight position
                    intakeBack.setPosition(0.5); //this is the initial position
                    // Position 3 (drop sample into bucket)
                    pressCount = 0; // Reset to cycle back to initial position
                }
            }

            previousBState = currentBState;

        }
    }
}

