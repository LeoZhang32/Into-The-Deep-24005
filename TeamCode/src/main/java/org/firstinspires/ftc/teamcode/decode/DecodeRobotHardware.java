/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named DecodeRobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class DecodeRobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor FL   = null;
    private DcMotor FR  = null;
    private DcMotor BL   = null;
    private DcMotor BR  = null;
    private IMU imu = null;
    private DcMotorEx shooter = null;
    private DcMotorEx intake =null;
    private Servo trigger = null;
    private Servo gate = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DecodeRobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        FL  = myOpMode.hardwareMap.get(DcMotor.class, "FL");
        FR = myOpMode.hardwareMap.get(DcMotor.class, "FR");
        BL  = myOpMode.hardwareMap.get(DcMotor.class, "BL");
        BR = myOpMode.hardwareMap.get(DcMotor.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "abc");
        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "def");

        trigger = myOpMode.hardwareMap.get(Servo.class, "trigger");
        gate = myOpMode.hardwareMap.get(Servo.class, "gate");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //TODO: add reversed motors
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive_y    y-direction power (-1.0 to 1.0)
     * @param Drive_x    x-direction power (-1.0 to 1.0)
     * @param Turn       Right/Left turning power (-1.0 to 1.0)
     * @param SlowModeOn slow mode boolean
     */
    public void driveRobot(double Drive_y, double Drive_x, double Turn, boolean SlowModeOn, boolean ImuReset) {
        double BotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (ImuReset){
            imu.resetYaw();
        }
        // Rotate the movement direction counter to the bot's rotation
        double rotX = Drive_x * Math.cos(-BotHeading) - Drive_y * Math.sin(-BotHeading);
        double rotY = Drive_x * Math.sin(-BotHeading) + Drive_y * Math.cos(-BotHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(Turn), 1);
        double frontLeftPower = (rotY + rotX + Turn) / denominator;
        double backLeftPower = (rotY - rotX + Turn) / denominator;
        double frontRightPower = (rotY - rotX - Turn) / denominator;
        double backRightPower = (rotY + rotX - Turn) / denominator;

        if (SlowModeOn){
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
    }
    /**
     * @param shooterInput -shooter button
     */
    public void shooterCycle (boolean shooterInput){
//        boolean velocityValid = false;
//        boolean velocityValid2 = false;
//        double shooterVelocity = 0;
//        shooterVelocity = shooter.getVelocity(AngleUnit.DEGREES);
//        if (shooterInput){
//            shooter.setPower(0.55);
//        }
//        else shooter.setPower(0);
//        myOpMode.telemetry.addData("velocity",shooterVelocity);
//        velocityValid = shooterVelocity >= 140;
//        if (velocityValid) trigger.setPosition(0.68);
//        else trigger.setPosition(1);
//        velocityValid2 = shooterVelocity >= 138;
//        if (velocityValid2) gate.setPosition(0.65);
//        else gate.setPosition(0.98);
//        myOpMode.telemetry.update();
    }

    /**
     *
     * @param intakeInInput -intake button
     * @param intakeOutInput -reverse intake button
     * @param outtakeOn -outtake button
     */
    public void intakeOuttakeAction(boolean intakeInInput, boolean intakeOutInput, boolean outtakeOn){
        double shooterVelocity = 0;
        shooterVelocity = shooter.getVelocity(AngleUnit.DEGREES);
        myOpMode.telemetry.addData("velocity",shooterVelocity);
        myOpMode.telemetry.update();

        boolean velocityValid = false;
        boolean velocityValid2 = false;

        velocityValid = shooterVelocity >= 145;
        if (velocityValid) trigger.setPosition(0.68);
        else trigger.setPosition(0.95);

        velocityValid2 = shooterVelocity >= 143;
        if (velocityValid2) gate.setPosition(0.65);
        else gate.setPosition(0.98);

        if (intakeInInput) intake.setPower(1);
        else if (intakeOutInput) intake.setPower(-1);
        else intake.setPower(0);

        if (!outtakeOn){
            shooter.setPower(0);
        }
        else {
            shooter.setPower(0.6);
        }
    }
}
