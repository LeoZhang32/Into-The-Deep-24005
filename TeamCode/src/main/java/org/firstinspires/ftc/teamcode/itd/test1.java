package org.firstinspires.ftc.teamcode.itd;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class test1 extends LinearOpMode{
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontViper;
    DcMotor backViper;
    Servo claw;
    Boolean claw_open = true;
    Boolean claw_button_pressed = false;
    Servo bucket;
    Boolean bucket_button_pressed = false;
    Boolean bucket_dumped = false;
    Servo specimen;
    Boolean specimen_button_pressed = false;
    Boolean specimen_closed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // drivetrain motors
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //viper slides motors
        frontViper = hardwareMap.dcMotor.get("frontViper");
        backViper = hardwareMap.dcMotor.get("backViper");
        frontViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        claw = hardwareMap.servo.get("claw");
        bucket = hardwareMap.servo.get("bucket");
        specimen = hardwareMap.servo.get("specimen");

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {

            //drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower * 0.75);
            backLeft.setPower(backLeftPower * 0.75);
            frontRight.setPower(frontRightPower * 0.75);
            backRight.setPower(backRightPower * 0.75);

            //viper slides
            if (gamepad1.dpad_up) {
                frontViper.setPower(-0.8);
                backViper.setPower(0.8);
            } else if (gamepad1.dpad_down) {
                frontViper.setPower(0.6);
                backViper.setPower(-0.6);
            } else {
                frontViper.setPower(0);
                backViper.setPower(0);
            }

            //sticky key presses
            if (gamepad1.a){
                if (!claw_button_pressed){
                    claw_open = !claw_open;
                }
                claw_button_pressed= true;
            } else claw_button_pressed = false;

            if (gamepad1.y) {
                if (!bucket_button_pressed) {
                    bucket_dumped = !bucket_dumped;
                }
                bucket_button_pressed = true;
            } else bucket_button_pressed = false;

            if (gamepad1.b) {
                if (!specimen_button_pressed) {
                    specimen_closed = !specimen_closed;
                }
                specimen_button_pressed = true;
            } else specimen_button_pressed = false;

            updateBooleans();}

        }
    public void updateBooleans() {
        if (claw_open){
             claw.setPosition(0.4);
             telemetry.addData("Pos","0.4");
             telemetry.update();
         }
        else{
             claw.setPosition(1);
            telemetry.addData("Pos","1");
            telemetry.update();
            }

        if (bucket_dumped){
            bucket.setPosition(0.55);
        }
        else {
            bucket.setPosition(1);
        }

        if (specimen_closed){
            specimen.setPosition(0.7);
        }
        else {
            specimen.setPosition(0.9);
    }
}}
