package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class motorstest extends LinearOpMode {

    Limelight3A limelight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

                if (gamepad1.y){
                    frontLeft.setPower(1);
                }
                else {
                    frontLeft.setPower(0);
                }

                if (gamepad1.b){
                    frontRight.setPower(1);
                }
                else {
                    frontRight.setPower(0);
                }

                if (gamepad1.x){
                    backLeft.setPower(1);
                }
                else{
                    backLeft.setPower(0);
                }

                if (gamepad1.a){
                    backRight.setPower(1);

                }
                else{
                    backRight.setPower(0);
                }
            telemetry.update();
        }
    }
}