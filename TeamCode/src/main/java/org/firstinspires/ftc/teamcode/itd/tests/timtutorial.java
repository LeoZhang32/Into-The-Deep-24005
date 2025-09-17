package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class timtutorial extends LinearOpMode {
DcMotor Tim;

    @Override
    public void runOpMode() throws InterruptedException {
        Tim = hardwareMap.get(DcMotor.class, "abc");
        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a){
                Tim.setPower(1);
            }
            else if (gamepad1.b){
                Tim.setPower(-1);
            }
            else if (gamepad1.x){
                Tim.setPower(0.5);
            }
            else if (gamepad1.y){
                Tim.setPower(-0.5);
            }
            else {
                Tim.setPower(0);
            }
        }
    }
}