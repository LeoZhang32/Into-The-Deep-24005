package org.firstinspires.ftc.teamcode.itd.nationals;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class teleop_new extends LinearOpMode {
    Servo IArmL;
    Servo IArmR;
    Servo IWrist;
    Servo IClaw;
    Servo IArmC;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    DcMotor VSlideF;
    DcMotor VSlideB;

    Boolean slowModeOn = false;
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;

    IMU pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

        positions_and_variables pos = new positions_and_variables();
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        IArmL = hardwareMap.get(Servo.class, "IArmL");
        IArmR = hardwareMap.get(Servo.class, "IArmR");
        IArmC = hardwareMap.get(Servo.class, "IArmC");
        IWrist = hardwareMap.get(Servo.class, "IWrist");
        IClaw = hardwareMap.get(Servo.class, "IClaw");

        double vert_power = pos.vert_slides_power;

        VSlideF = hardwareMap.get(DcMotor.class, "VSlideF");
        VSlideB = hardwareMap.get(DcMotor.class, "VSlideB");
        VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);


        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint = hardwareMap.get(IMU.class, "Pinpoint");
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        pinpoint.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.start){
                pinpoint.resetYaw();
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;
            double botHeading = pinpoint.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

            if (slowModeOn){
                FL.setPower(frontLeftPower * 0.5);
                BL.setPower(backLeftPower * 0.5);
                FR.setPower(frontRightPower * 0.5);
                BR.setPower(backRightPower * 0.5);
            }
            else{
                FL.setPower(frontLeftPower * 1);
                BL.setPower(backLeftPower * 1);
                FR.setPower(frontRightPower * 1);
                BR.setPower(backRightPower * 1);
            }
            if (gamepad2.dpad_up){
                VSlideF.setPower(vert_power);
                VSlideB.setPower(-vert_power);
            }
            else {
                VSlideF.setPower(0);
                VSlideB.setPower(0);
            }
            if (gamepad2.dpad_up){
                VSlideF.setPower(-vert_power);
                VSlideB.setPower(vert_power);
            }
            else {
                VSlideF.setPower(0);
                VSlideB.setPower(0);
            }
            cycle_gamepad1.updateX(4);
            cycle_gamepad1.updateRB(4);
            cycle_gamepad1.updateA(2);

            dashboard.sendTelemetryPacket(packet);
            //arm movements

            if (cycle_gamepad1.xPressCount == 0){
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
            }
            else if (cycle_gamepad1.xPressCount == 1){
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
            }
            else if (cycle_gamepad1.xPressCount == 2){
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
            }
            else{
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
            }
            //wrist movements
            if (cycle_gamepad1.rbPressCount == 0){
                IWrist.setPosition(pos.intake_wrist0);
            }
            else if (cycle_gamepad1.rbPressCount == 1){
                IWrist.setPosition(pos.intake_wrist1);
            }
            else if (cycle_gamepad1.rbPressCount == 2){
                IWrist.setPosition(pos.intake_wrist2);
            }
            else{
                IWrist.setPosition(pos.intake_wrist3);
            }
            //claw movement
            if (cycle_gamepad1.aPressCount == 0){
                IClaw.setPosition(pos.intake_claw_open);
            }
            else {
                IClaw.setPosition(pos.intake_claw_close);
            }
        }
    }
}
