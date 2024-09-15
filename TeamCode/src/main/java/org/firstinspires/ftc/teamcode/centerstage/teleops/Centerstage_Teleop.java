package org.firstinspires.ftc.teamcode.centerstage.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Centerstage_Teleop extends LinearOpMode {
    IMU imu;
    DcMotor intake;
    DcMotor rightViper;
    DcMotor leftViper;
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    Servo claw;
    Servo claw_wrist;
    Servo claw_elbow;
    Servo drone;
    Servo intake_drop;
    //booleans
    Double[] wrist_positions = {0.0, 0.2, 0.3, 0.4};
    Integer x = 0;
    Boolean elbow_position_score = false;
    Boolean elbow_button_pressed = false;
    Boolean claw_open = true;
    Boolean claw_button_pressed = false;
    Boolean drone_button_pressed = false;
    Boolean drone_launched = false;
    Boolean wristHoriz = false;
    Boolean wristPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        leftViper = hardwareMap.dcMotor.get("leftViper");
        rightViper = hardwareMap.dcMotor.get("rightViper");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        claw = hardwareMap.servo.get("claw");
        claw_wrist = hardwareMap.servo.get("claw_wrist");
        claw_elbow = hardwareMap.servo.get("claw_elbow");
        drone = hardwareMap.servo.get("drone");
        intake_drop = hardwareMap.servo.get("intake_drop");
        imu = hardwareMap.get(IMU.class, "imu");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()&&opModeIsActive()) {
            //drivetrain
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(0.75 * frontLeftPower);
            backLeftMotor.setPower(0.75 * backLeftPower);
            frontRightMotor.setPower(0.75 * frontRightPower);
            backRightMotor.setPower(0.75 * backRightPower);

            //intake
            if (gamepad1.right_bumper) {
                intake_drop.setPosition(0.9);
                intake.setPower(0.45);
            } else if (gamepad1.left_bumper) {
                intake_drop.setPosition(0.9);
                intake.setPower(-0.45);
            } else {
                intake_drop.setPosition(0.4);
                intake.setPower(0);
            }

            //viper slides
            if (gamepad2.dpad_up) {
                leftViper.setPower(1);
                rightViper.setPower(-1);
            } else if (gamepad2.dpad_down) {
                leftViper.setPower(-1);
                rightViper.setPower(1);
            } else {
                leftViper.setPower(0);
                rightViper.setPower(0);
            }

            //Sticky key presses
            if (gamepad2.y) {
                if (!drone_button_pressed) {
                    drone_launched = !drone_launched;
                }
                drone_button_pressed = true;
            } else drone_button_pressed = false;
            if (gamepad2.x){
                if (!elbow_button_pressed) {
                    elbow_position_score = !elbow_position_score;
                }
                elbow_button_pressed = true;
            } else elbow_button_pressed = false;
            if (gamepad2.a){
                if (!claw_button_pressed){
                    claw_open = !claw_open;
                }
                claw_button_pressed= true;
            } else claw_button_pressed = false;
            if (elbow_position_score){
                if (gamepad2.b){
                    if (!wristPressed){
                    wristHoriz = !wristHoriz;
                    }
                wristPressed= true;
                } else wristPressed = false;
            }
            updateBooleans();

        }
    }
    public void updateBooleans() {
        if (drone_launched){
            drone.setPosition(0.7);
        }
        else {
            drone.setPosition(0);
        }
        if (elbow_position_score){
            claw_elbow.setPosition(0.45);
        }
        else {
            claw_elbow.setPosition(0.225);
            wristHoriz = Boolean.FALSE;
        }
        if (claw_open){
            claw.setPosition(0);
        }
        else{
            claw.setPosition(0.5);
        }
        if (wristHoriz){
            claw_wrist.setPosition(0.35);
        }
        else{
            claw_wrist.setPosition(0);
        }
    }
}
