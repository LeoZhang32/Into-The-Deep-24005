package org.firstinspires.ftc.teamcode.centerstage.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class old_teleop extends LinearOpMode {
    Servo claw;
    Servo clawPivot;
    Servo droneLauncher;
    Servo hookTurn;
    DcMotor intake;
    DcMotor ViperSlideRight;
    DcMotor hangPulley;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Boolean clawOpen = true;
    Boolean aIsPressed = false;
    Boolean rightBumperIsPressed = false;
    Boolean invertControlsOn = false;
    Boolean yIsPressed = false;
    Boolean droneLaunched = false;
    Boolean leftBumperIsPressed = false;
    Boolean slowModeOn = false;
    Boolean bIsPressed = false;
    Boolean clawScorePosition = false;
    Boolean xIsPressed = false;
    Boolean hookReady = false;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        ViperSlideRight = hardwareMap.dcMotor.get("ViperSlideRight");
        hangPulley = hardwareMap.dcMotor.get("hangPulley");
        claw = hardwareMap.servo.get("claw");
        clawPivot = hardwareMap.servo.get("clawPivot");
        droneLauncher = hardwareMap.servo.get("droneLauncher");
        hookTurn = hardwareMap.servo.get("hookTurn");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //modify this yourself base on what direction each wheel is spinning

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if (gamepad1.left_bumper) {
                if (!leftBumperIsPressed) {
                    slowModeOn = !slowModeOn;
                }
                leftBumperIsPressed = true;
            } else leftBumperIsPressed = false;
            if (gamepad1.right_bumper) {
                if (!rightBumperIsPressed) {
                    invertControlsOn = !invertControlsOn;
                }
                rightBumperIsPressed = true;
            } else rightBumperIsPressed = false;

            if (slowModeOn) {
                frontLeftMotor.setPower(frontLeftPower * 0.25);
                backLeftMotor.setPower(backLeftPower * 0.25);
                frontRightMotor.setPower(frontRightPower * 0.25);
                backRightMotor.setPower(backRightPower * 0.25);
                if (invertControlsOn) {
                    frontLeftMotor.setPower(-frontLeftPower);
                    backLeftMotor.setPower(-backLeftPower);
                    frontRightMotor.setPower(-frontRightPower);
                    backRightMotor.setPower(-backRightPower);
                }
            } else if (invertControlsOn) {
                frontLeftMotor.setPower(-frontLeftPower);
                backLeftMotor.setPower(-backLeftPower);
                frontRightMotor.setPower(-frontRightPower);
                backRightMotor.setPower(-backRightPower);
            } else {
                frontLeftMotor.setPower(frontLeftPower * 0.6);
                backLeftMotor.setPower(backLeftPower * 0.6);
                frontRightMotor.setPower(frontRightPower * 0.6);
                backRightMotor.setPower(backRightPower * 0.6);
            }
            if (gamepad1.x) {
                intake.setPower(0.4);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.y){
                intake.setPower(-0.4);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.b) {
                hangPulley.setPower(0.5);
            } else {
                hangPulley.setPower(0);
            }
            if (gamepad2.dpad_up) {
                ViperSlideRight.setPower(-0.75);
            } else {
                ViperSlideRight.setPower(0);
            }
            if (gamepad2.dpad_down) {
                ViperSlideRight.setPower(0.5);
            } else {
                ViperSlideRight.setPower(0);
            }
            if (gamepad2.a || gamepad1.a) {
                if (!aIsPressed) {
                    clawOpen = !clawOpen;
                }
                aIsPressed = true;

            } else aIsPressed = false;

            if (gamepad2.b) {
                if (!bIsPressed) {
                    clawScorePosition = !clawScorePosition;
                }
                bIsPressed = true;

            } else bIsPressed = false;

            if (gamepad2.y) {
                if (!yIsPressed) {
                    droneLaunched = !droneLaunched;
                }
                yIsPressed = true;
            } else yIsPressed = false;

            if (gamepad2.x) {
                if (!xIsPressed) {
                    hookReady = !hookReady;
                }
                xIsPressed = true;
            } else xIsPressed = false;
            updateBooleans();
        }
    }
    public void updateBooleans(){
        if(clawOpen){
            claw.setPosition(0.8); //Change this bound.
        }else{
            claw.setPosition(2.5);
        }
        if (droneLaunched){
            droneLauncher.setPosition(0.7);
        }
        else{
            droneLauncher.setPosition(0);
        }
        if (clawScorePosition) {
            clawPivot.setPosition(0.3);
        }
        else {
            clawPivot.setPosition(0.05);
        }
        if (hookReady) {
            hookTurn.setPosition(0.01);
            telemetry.addData("hook position","0.02");
            telemetry.update();
        }
        else {
            hookTurn.setPosition(0.335);
            telemetry.addData("hook position","0.32");
            telemetry.update();
        }
    }
}