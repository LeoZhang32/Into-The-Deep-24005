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
public class limelight_dt_test3 extends LinearOpMode {

    Limelight3A limelight;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FL = hardwareMap.get(DcMotor.class,"FL");
        FR = hardwareMap.get(DcMotor.class,"FR");
        BL = hardwareMap.get(DcMotor.class,"BL");
        BR = hardwareMap.get(DcMotor.class,"BR");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

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

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.update();

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    if (result.getTy() < -5) {
                        FL.setPower(-0.2);
                        BR.setPower(-0.2);
                        FR.setPower(0.2);
                        BL.setPower(0.2);
                    } else if (result.getTy() > 5) {
                        FL.setPower(0.2);
                        BR.setPower(0.2);
                        FR.setPower(-0.2);
                        BL.setPower(-0.2);
                    } else {
                        if (result.getTx() > 5) {
                            FL.setPower(-0.2);
                            BR.setPower(-0.2);
                            FR.setPower(-0.2);
                            BL.setPower(-0.2);
                        } else if (result.getTx() < -5) {
                            FL.setPower(0.2);
                            BR.setPower(0.2);
                            FR.setPower(0.2);
                            BL.setPower(0.2);
                        } else {
                            FL.setPower(0);
                            BR.setPower(0);
                            FR.setPower(0);
                            BL.setPower(0);
                        }
                    }
                }
            }


//                else {
//                    telemetry.addData("result is not valid", result);
//                    telemetry.update();
//
//                }
//            } else {
//                telemetry.addData("no result", result);
//                telemetry.update();
//            }
            telemetry.update();

//            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//            telemetry.addData("Limelight Connected", limelight != null ? "Yes" : "No");
//            telemetry.update();
        }
    }
}