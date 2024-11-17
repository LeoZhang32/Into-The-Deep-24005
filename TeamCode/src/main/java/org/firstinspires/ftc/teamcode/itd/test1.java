package org.firstinspires.ftc.teamcode.itd;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp
public class test1 extends LinearOpMode{
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor frontViper;
    DcMotor backViper;

    Servo sample;
    Boolean sample_button_pressed = false;
    Boolean sample_closed = true;

    Servo intakeRight;
//    Boolean intakeRight_button_pressed = false;
//    Boolean intakeRight_extended = false;

    Servo intakeLeft;
//    Boolean intakeLeft_button_pressed = false;
//    Boolean intakeLeft_extended = false;

    Boolean intake_button_pressed = false;
    Boolean intake_down;

    Servo intakeBack;
//    Boolean intakeBack_button_pressed = false;
//    Boolean intakeBack_extended = false;

    Servo bucket;
    Boolean bucket_button_pressed = false;
    Boolean bucket_dumped = false;

    Servo specimen;
    Boolean specimen_button_pressed = false;
    Boolean specimen_closed = false;
;
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
        sample = hardwareMap.servo.get("sample");
        intakeRight = hardwareMap.servo.get("intakeRight");
        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeBack = hardwareMap.servo.get("intakeBack");
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
            //sample
            if (gamepad1.a){
                if (!sample_button_pressed){
                    sample_closed = !sample_closed;
                }
                sample_button_pressed= true;
            } else sample_button_pressed = false;

            //intakeRight
//            if (gamepad1.x){
//                if (!intakeRight_button_pressed){
//                    intakeRight_extended = !intakeRight_extended;
//                }
//                intakeRight_button_pressed= true;
//            } else intakeRight_button_pressed = false;
//
//            //intakeLeft
//            if (gamepad1.x){
//                if (!intakeLeft_button_pressed){
//                    intakeLeft_extended = !intakeLeft_extended;
//                }
//                intakeLeft_button_pressed= true;
//            } else intakeLeft_button_pressed = false;
//
//            //intakeBack
//            if (gamepad1.dpad_right){
//                if (!intakeBack_button_pressed){
//                    intakeBack_extended = !intakeBack_extended;
//                }
//                intakeBack_button_pressed = true;
//            } else intakeBack_button_pressed = false;
//
            //intake
            if (gamepad1.x){
                if (!intake_button_pressed){
                    intake_down = !intake_down;
                }
                intake_button_pressed = true;
            }else intake_button_pressed = false;

            //bucket
            if (gamepad1.y) {
                if (!bucket_button_pressed) {
                    bucket_dumped = !bucket_dumped;
                }
                bucket_button_pressed = true;
            } else bucket_button_pressed = false;

            //specimen
            if (gamepad1.b) {
                if (!specimen_button_pressed) {
                    specimen_closed = !specimen_closed;
                }
                specimen_button_pressed = true;
            } else specimen_button_pressed = false;

            updateBooleans();}

        }
    public void updateBooleans() {
        if (sample_closed){
            sample.setPosition(0.4);//this is the initial position
         }
        else{
            sample.setPosition(1);
            }

//        if (intakeRight_extended){
//            intakeRight.setPosition(0.16);
//        }
//        else {
//            intakeRight.setPosition(0.43); //this is the initial position
//             }
//        if (intakeLeft_extended){
//            intakeLeft.setPosition(0.84); //this = 1-intakeRight position
//        }
//        else {
//            intakeLeft.setPosition(0.57); //this = 1-intakeRight position
//        }
//
//        if (intakeBack_extended){
//            intakeBack.setPosition(0.97);
//                }
//        else {
//            intakeBack.setPosition(0.6); //this is the initial position
//                }
        if (intake_down){
            //down position
            intakeRight.setPosition(0.16);
            intakeLeft.setPosition(0.84);
            intakeBack.setPosition(0.97);
        }
        else {
            //initial position
            intakeRight.setPosition(0.43);
            intakeLeft.setPosition(0.57);
            intakeBack.setPosition(0.6);
        }
        if (bucket_dumped){
            bucket.setPosition(0.55);
        }
        else {
            bucket.setPosition(1);
        }

        if (specimen_closed){
            specimen.setPosition(0.67);
            telemetry.addData("specimenPos","0.67");
            telemetry.update();
        }
        else {
            specimen.setPosition(0.8); //this is the initial position
            telemetry.addData("specimenPos","0.8");
            telemetry.update();
    }
}}
