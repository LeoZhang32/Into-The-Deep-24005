package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;

@TeleOp
public class shooter_1 extends LinearOpMode {
DcMotor intake;
DcMotor outtake;
Servo trigger;

ElapsedTime triggerTimer = new ElapsedTime();
boolean isTriggerTimerRunning = false; // Track if timer is running

    @Override
    public void runOpMode() throws InterruptedException {
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);
        intake = hardwareMap.get(DcMotor.class, "abc");
        outtake = hardwareMap.get(DcMotor.class, "def");
        trigger = hardwareMap.get(Servo.class, "trigger");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            cycle_gamepad1.updateX(6);

            if (cycle_gamepad1.xPressCount == 0){
                intake.setPower(0);
                outtake.setPower(0);
                trigger.setPosition(0.7);
                isTriggerTimerRunning = false;
            }

            else if (cycle_gamepad1.xPressCount == 1){
                intake.setPower(1);
                outtake.setPower(0);
                trigger.setPosition(0.7);
                isTriggerTimerRunning = false;
            }

            else if (cycle_gamepad1.xPressCount == 2){
                intake.setPower(0);
                outtake.setPower(0);
                trigger.setPosition(0.7);
                isTriggerTimerRunning = false;
            }

            else if (cycle_gamepad1.xPressCount == 3){
                intake.setPower(0);
                outtake.setPower(0.8);
                trigger.setPosition(0.7);
                isTriggerTimerRunning = false;
            }

            else if (cycle_gamepad1.xPressCount == 4){
                intake.setPower(1);
                outtake.setPower(0.8);
                trigger.setPosition(1);
                isTriggerTimerRunning = false;
                if (!isTriggerTimerRunning) {
                    triggerTimer.reset();
                    isTriggerTimerRunning = true; // Indicate timer has started
                    telemetry.addData("trigger Timer Started", triggerTimer.milliseconds());
                }
            }
            else {
                intake.setPower(0);
                outtake.setPower(0);
                trigger.setPosition(0.7);
                isTriggerTimerRunning = false;
            }

            //trigger action timer
            if (isTriggerTimerRunning) {
                telemetry.addData("Grab Timer Running", triggerTimer.milliseconds()); // Track progress

                if (triggerTimer.milliseconds() >= 1500) {
                    telemetry.addData("Grab Timer Expired", triggerTimer.milliseconds());

                    if (cycle_gamepad1.xPressCount == 4) {
                        cycle_gamepad1.xPressCount = 3;
                        isTriggerTimerRunning = false; // Stop tracking timer once done
                    }

                }
            }

            if (gamepad1.y) {
                cycle_gamepad1.xPressCount = 1;
            }

        }
    }
}