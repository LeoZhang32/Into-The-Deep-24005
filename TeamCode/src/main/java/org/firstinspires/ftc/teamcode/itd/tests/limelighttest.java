package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class limelighttest extends LinearOpMode {

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
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    if (result.getTx() < -10){
                        frontLeft.setPower(-0.2);
                        backRight.setPower(-0.2);
                        frontRight.setPower(0.2);
                        backLeft.setPower(0.2);
                    }
                    else if (result.getTx() > 10){
                        frontLeft.setPower(0.2);
                        backRight.setPower(0.2);
                        frontRight.setPower(-0.2);
                        backLeft.setPower(-0.2);
                    }
                    else {
                        if (result.getTy() > 10){
                            frontLeft.setPower(0.2);
                            backRight.setPower(0.2);
                            frontRight.setPower(0.2);
                            backLeft.setPower(0.2);
                        }
                        else if (result.getTy() < -10){
                            frontLeft.setPower(-0.2);
                            backRight.setPower(-0.2);
                            frontRight.setPower(-0.2);
                            backLeft.setPower(-0.2);
                        }
                        else {
                            frontLeft.setPower(0);
                            backRight.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                        }
                    }
                }
            }
            telemetry.update();
        }
    }
}