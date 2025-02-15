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
public class limelight_dt_test4 extends LinearOpMode {

    Limelight3A limelight;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    Boolean move_to_left_button_pressed = false;
    Boolean move_to_left = false;
    Boolean move_to_right_button_pressed = false;
    Boolean move_to_right = false;
    Boolean move_to_front_button_pressed = false;
    Boolean move_to_front = false;
    Boolean move_to_back_button_pressed = false;
    Boolean move_to_back = false;

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

            if (gamepad1.x) {
                if (!move_to_left_button_pressed) {
                    move_to_left = !move_to_left;
                }
                move_to_left_button_pressed = true;
            } else move_to_left_button_pressed = false;
            if (move_to_left){
                FL.setPower(-0.5);
                BR.setPower(-0.5);
                FR.setPower(0.5);
                BL.setPower(0.5);
                telemetry.addData("move to left", move_to_left);
                telemetry.addData("FL Power", FL.getPower());
                telemetry.addData("BR Power", BR.getPower());
                telemetry.addData("FR Power", FR.getPower());
                telemetry.addData("BL Power", BL.getPower());
                telemetry.update();
                } else {
                FL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
            }


            if (gamepad1.b) {
                if (!move_to_right_button_pressed) {
                    move_to_right = !move_to_right;
                }
                move_to_right_button_pressed = true;
            } else move_to_right_button_pressed = false;
            if (move_to_right){
                FL.setPower(0.5);
                BR.setPower(0.5);
                FR.setPower(-0.5);
                BL.setPower(-0.5);
                telemetry.addData("move to right", move_to_right);
                telemetry.addData("FL Power", FL.getPower());
                telemetry.addData("BR Power", BR.getPower());
                telemetry.addData("FR Power", FR.getPower());
                telemetry.addData("BL Power", BL.getPower());
                telemetry.update();
            }else {
                FL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
            }


            if (gamepad1.y) {
                if (!move_to_front_button_pressed) {
                    move_to_front = !move_to_front;
                }
                move_to_front_button_pressed = true;
            } else move_to_front_button_pressed = false;
            if (move_to_front){
                FL.setPower(0.5);
                BR.setPower(0.5);
                FR.setPower(0.5);
                BL.setPower(0.5);
                telemetry.addData("move to front", move_to_front);
                telemetry.addData("FL Power", FL.getPower());
                telemetry.addData("BR Power", BR.getPower());
                telemetry.addData("FR Power", FR.getPower());
                telemetry.addData("BL Power", BL.getPower());
                telemetry.update();
            }else {
                FL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
            }

            if (gamepad1.a) {
                if (!move_to_back_button_pressed) {
                    move_to_back = !move_to_back;
                }
                move_to_back_button_pressed = true;
            } else move_to_back_button_pressed = false;


            if (move_to_back){
                FL.setPower(-0.5);
                BR.setPower(-0.5);
                FR.setPower(-0.5);
                BL.setPower(-0.5);
                telemetry.addData("move to back", move_to_back);
                telemetry.addData("FL Power", FL.getPower());
                telemetry.addData("BR Power", BR.getPower());
                telemetry.addData("FR Power", FR.getPower());
                telemetry.addData("BL Power", BL.getPower());
                telemetry.update();
            } else {
                FL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
            }



            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.update();

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    if (result.getTy() < -2) {
                        FL.setPower(0.8);
                        BR.setPower(0.8);
                        FR.setPower(-0.8);
                        BL.setPower(-0.8);
                    } else if (result.getTy() > 2) {
                        FL.setPower(-0.8);
                        BR.setPower(-0.8);
                        FR.setPower(0.8);
                        BL.setPower(0.8);
                    } else {
                        if (result.getTx() > 2) {
                            FL.setPower(0.8);
                            BR.setPower(0.8);
                            FR.setPower(0.8);
                            BL.setPower(0.8);
                        } else if (result.getTx() < -2) {
                            FL.setPower(-0.8);
                            BR.setPower(-0.8);
                            FR.setPower(-0.8);
                            BL.setPower(-0.8);
                        } else {
                            FL.setPower(0);
                            BR.setPower(0);
                            FR.setPower(0);
                            BL.setPower(0);
                        }
                    }
                }
            }

            telemetry.update();

        }
    }
}