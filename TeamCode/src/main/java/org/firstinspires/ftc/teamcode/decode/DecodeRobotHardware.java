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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
    public static double outtakePower = 0.6;
    public static double triggerThreshold = 140;
    public static double triggerPosition = 0.68;

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

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 24.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.05  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0) pr 0.02
    final double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0) pr 0.015
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0) pr 0.01

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
  // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DecodeRobotHardware (LinearOpMode opmode) { myOpMode = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }


        if (visionPortal.getCameraState() != null){
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    myOpMode.sleep(20);
                }
            }
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested()){
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            myOpMode.sleep(20);
        }


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

    /** @param autoInput button to activate auto
     *  @param desiredTagID Tag ID
     **/
    public void driveToApril (boolean autoInput, double desiredTagID){
        double Drive_x = 0;
        double Drive_y = 0;
        double Turn = 0;
        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            myOpMode.telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            myOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            myOpMode.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            myOpMode.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            myOpMode.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            myOpMode.telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound && autoInput) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            Drive_y  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            Turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            Drive_x = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            myOpMode.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", Drive_y, Drive_x, Turn);
            double leftFrontPower    =  -(Drive_x -Drive_y -Turn);
            double rightFrontPower   =  -(Drive_x +Drive_y +Turn);
            double leftBackPower     =  -(Drive_x +Drive_y -Turn);
            double rightBackPower    =  -(Drive_x -Drive_y +Turn);

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max < -1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);
        }
        myOpMode.telemetry.update();
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
