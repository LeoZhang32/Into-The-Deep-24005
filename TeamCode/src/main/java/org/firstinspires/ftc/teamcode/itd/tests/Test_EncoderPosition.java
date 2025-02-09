package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Test_EncoderPosition extends LinearOpMode {
    DcMotor VSlideF;
    DcMotor VSlideB;


    @Override
    public void runOpMode() throws InterruptedException {

        VSlideF = hardwareMap.dcMotor.get("VSlideF");
        VSlideB = hardwareMap.dcMotor.get("VSlideB");


        // Reset the encoder during initialization
        VSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        VSlideF.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            telemetry.addData("position", VSlideF.getCurrentPosition());
            telemetry.addData("position", VSlideB.getCurrentPosition());
            telemetry.update();
        }
    }
}
