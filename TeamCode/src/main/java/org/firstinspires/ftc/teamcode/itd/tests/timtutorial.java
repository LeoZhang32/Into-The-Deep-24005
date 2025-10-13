package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class timtutorial extends LinearOpMode {
DcMotor Tim;
DcMotor Ryan;
//Servo Trigger;

    @Override
    public void runOpMode() throws InterruptedException {
        Tim = hardwareMap.get(DcMotor.class, "abc");
        Ryan = hardwareMap.get(DcMotor.class, "def");
//        Trigger = hardwareMap.get(Servo.class, "trigger");

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a){
                Tim.setPower(1);
            }
            else if (gamepad1.b){
                Tim.setPower(-1);
            }
            else Tim.setPower(0);
            if (gamepad1.x){
                Ryan.setPower(0.7);
            }
            else if (gamepad1.y){
                Ryan.setPower(0.8);
            }
            else if (gamepad1.right_bumper){
                Ryan.setPower(0.6);
            }
            else if (gamepad1.left_bumper){
                Ryan.setPower(0.5);
            }
            else if (gamepad1.dpad_up){
                Ryan.setPower(1.0);
            }
            else if (gamepad1.dpad_down){
                Ryan.setPower(0.9);
            }

            else {
                Ryan.setPower(0);
            }
        }
    }
}