package org.firstinspires.ftc.teamcode.itd.tests;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class teleop_20241222 extends LinearOpMode{
    //drivetrain
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    IMU imu;

    // Drivetrain class
    // Declare PID constants and variables
    private double targetHeading;  // Desired heading in radians
    private final double kP = 0.15;    // Proportional gain (start small and tune this value)
    private final double kI = 0;      // Integral gain (optional, set to 0 initially)
    private final double kD = 0;      // Derivative gain (optional, set to 0 initially)
    private double lastHeadingError = 0;
    private double headingIntegral = 0;

    //viper slides
    DcMotor frontViper;
    DcMotor backViper;
    Boolean VS_manual_running = false;
    Boolean VS_auto_running = false;
    Boolean VS_auto_up_button_pressed = false;
    Boolean VS_auto_up = false;
    Boolean VS_auto_down_button_pressed = false;
    Boolean VS_auto_down = false;
    Boolean VS_specscore_button_pressed = false;
    Boolean VS_specscore = false;

    //servos
    Servo sample;
    Boolean sample_button_pressed = false;
    Boolean sample_closed = false;
//    private ElapsedTime claw_time = new ElapsedTime();


    Servo intakeRight;
    Servo intakeLeft;
    Servo intakeBack;
    int pressCount = 0;

    Boolean previousXState = false;
    Boolean currentXState;
    Boolean previousYState = false;

    Boolean currentYState;
    Boolean previousBState = false;
    Boolean currentBState;
    Boolean previousRBState = false;
    Boolean currentRBState;

    Servo bucket;
    Boolean bucket_button_pressed = false;
    Boolean bucket_dumped = false;

    Servo specimen;
    Boolean specimen_button_pressed = false;
    Boolean specimen_closed = false;

    Servo hangRight;
    Boolean hangRight_button_pressed = false;
    Boolean hangRight_activated = false;

    Servo hangLeft;
    Boolean hangLeft_button_pressed = false;
    Boolean hangLeft_activated = false;

    NormalizedColorSensor colorSensor;
    View relativeLayout;
    Boolean sample_color = false;

    Boolean slowModeOn = false;
    Boolean slowModeButtonPressed = false;

    Boolean specimenModeOn = false;
    Boolean specimenModeButtonPressed = false;

    DigitalChannel limitSwitch;
//    Boolean gamepad2_dpad_down_isPressed = false;

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
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //viper slides motors
        frontViper = hardwareMap.dcMotor.get("frontViper");
        backViper = hardwareMap.dcMotor.get("backViper");

        frontViper.setDirection(DcMotor.Direction.REVERSE);

        // Reset the encoder during initialization
        frontViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        sample = hardwareMap.servo.get("sample");
        intakeRight = hardwareMap.servo.get("intakeRight");
        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeBack = hardwareMap.servo.get("intakeBack");
        bucket = hardwareMap.servo.get("bucket");
        specimen = hardwareMap.servo.get("specimen");
        hangRight = hardwareMap.servo.get("hangRight");
        hangLeft = hardwareMap.servo.get("hangLeft");


        // Set the initial positions for intakeRight, intakeLeft and intakeBack
        intakeRight.setPosition(0.57);
        intakeLeft.setPosition(0.43);
        intakeBack.setPosition(0.45);

        //sensors
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }
    protected void runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.


        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {


            telemetry.addData("frontViper position", frontViper.getCurrentPosition());
            telemetry.addData("backViper position", backViper.getCurrentPosition());
            telemetry.update();

            //drivetrain
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
                targetHeading = 0;  // Reset to zero after resetting the IMU
            }


            // Get current heading from the IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // Manual turning detection
            if (Math.abs(rx) > 0.05) { // Threshold to detect intentional turning
                // Driver is manually turning; update target heading to current heading
                targetHeading = botHeading;
            }


            // PID correction for heading
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double headingError = targetHeading - currentHeading;

            // Normalize error to the range [-π, π] for smoother corrections
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

            // Apply PID only if not manually turning
            double correction = 0;
            if (Math.abs(rx) <= 0.05) { // Only correct heading when not manually turning
                // Proportional term
                correction = headingError * kP;

                // Optional: Add Integral and Derivative terms if needed
                headingIntegral += headingError;  // Accumulate the error over time
                double headingDerivative = headingError - lastHeadingError;
                correction += (headingIntegral * kI) + (headingDerivative * kD);

                // Update last error
                lastHeadingError = headingError;
            }

            // Apply correction to rotational power
            rx -= correction;





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
                frontLeft.setPower(frontLeftPower * 0.3);
                backLeft.setPower(backLeftPower * 0.3);
                frontRight.setPower(frontRightPower * 0.3);
                backRight.setPower(backRightPower * 0.3);
            }
            else{
                frontLeft.setPower(frontLeftPower * 1);
                backLeft.setPower(backLeftPower * 1);
                frontRight.setPower(frontRightPower * 1);
                backRight.setPower(backRightPower * 1);
            }



            // Add telemetry for debugging
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Correction", correction);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();

            if (gamepad2.x){
                VS_auto_running = true;
                VS_auto_up = true;
            }
            if (gamepad2.b){
                VS_auto_running = true;
                VS_auto_down = true;
            }
            if (gamepad2.right_bumper || gamepad2.left_bumper){
                VS_auto_running = true;
                VS_specscore = true;
            }

            viper_slides: if (gamepad2.dpad_up){
                VS_manual_running = true;
                frontViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontViper.setPower(1);
                backViper.setPower(1);
                if (frontViper.getCurrentPosition() >= 4200 || backViper.getCurrentPosition() >= 4200){
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    break viper_slides;
                }
            }
            else if (gamepad2.dpad_down){
                VS_manual_running = true;
                frontViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontViper.setPower(-1);
                backViper.setPower(-1);
                if (!limitSwitch.getState() || frontViper.getCurrentPosition() <= 50 || backViper.getCurrentPosition() <= 50){
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    telemetry.addData("viper slides","stopped");
                    telemetry.update();
                    break viper_slides;
                }

            }
            else if (VS_auto_running){
                if (specimenModeOn){
                    if (VS_auto_up){
                        if (gamepad2.start){
                        frontViper.setPower(0);
                        backViper.setPower(0);
                        break viper_slides;
                        }
                        frontViper.setTargetPosition(2150);
                        backViper.setTargetPosition(2150);
                        frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (frontViper.getCurrentPosition() == 2150 || backViper.getCurrentPosition() == 2150){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            VS_auto_up = false;
                            break viper_slides;
                        }
                    }
                    else if (VS_auto_down){
                        if (gamepad2.start){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            break viper_slides;
                        }
                        frontViper.setTargetPosition(260);
                        backViper.setTargetPosition(260);
                        frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (frontViper.getCurrentPosition() == 260 || backViper.getCurrentPosition() == 260){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            VS_auto_down = false;
                            break viper_slides;
                        }
                    }
                    else if (VS_specscore){
                        if (gamepad2.start){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            break viper_slides;
                        }
                        frontViper.setTargetPosition(1600);
                        backViper.setTargetPosition(1600);
                        frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (frontViper.getCurrentPosition() == 1600 || backViper.getCurrentPosition() == 1600){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            VS_specscore = false;
                            break viper_slides;
                        }

                    }
                }
                else {
                    if (VS_auto_up){
                        if (gamepad2.start){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            break viper_slides;
                        }
                        frontViper.setTargetPosition(4200);
                        backViper.setTargetPosition(4200);
                        frontViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        backViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (frontViper.getCurrentPosition() == 4200 || backViper.getCurrentPosition() == 4200){
                            frontViper.setPower(0);
                            backViper.setPower(0);
                            VS_auto_up = false;
                            break viper_slides;
                        }
                    }
                }
            }
            else if (!limitSwitch.getState() && !VS_manual_running && !VS_auto_running){
                frontViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            sticky_keys();
            updateBooleans();

            // Explain basic gain information via telemetry
            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.setGain(gain);

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }
            if (hsvValues[0] == 0){
                continue;
            }
            else if (hsvValues[0] >= -10 && hsvValues[0] <= 50) {
                sample_color = true;
            }
            else if (hsvValues[0] >= 200 && hsvValues[0] <=280){
                sample_color = false;
                telemetry.addData("Color is Blue", "Sample claw doesn't close.");
            }
            else {
                sample_color = false;
                telemetry.addData("Color Is Not Seen", "Sample claw doesn't close");
            }

            telemetry.update();
//            if (sample_color){
//                sample.setPosition(1);
//                telemetry.addData("sample_color", "true");
//            }
//            else {
//                sample.setPosition(0.4);
//                telemetry.addData("sample_color", "false");
//            }
            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });


        }

    }

    public void sticky_keys() {
        //sticky key presses
        //specimen mode
        if (gamepad2.start&& gamepad2.back){
            if (!specimenModeButtonPressed){
                specimenModeOn = !specimenModeOn;
            }
            specimenModeButtonPressed= true;
        } else specimenModeButtonPressed = false;
        //slow mode
        if (gamepad1.left_bumper){
            if (!slowModeButtonPressed){
                slowModeOn = !slowModeOn;
            }
            slowModeButtonPressed= true;
        } else slowModeButtonPressed = false;
        //sample
        if (gamepad1.a){
            if (!sample_button_pressed){
                sample_closed = !sample_closed;
            }
            sample_button_pressed= true;
        } else sample_button_pressed = false;

        //intakeActions
        currentXState = gamepad1.x;
        currentYState = gamepad1.y;
        currentBState = gamepad1.b;
        currentRBState = gamepad1.right_bumper;

        if (currentXState && !previousXState) { // Prevent "button held down" behavior
            // Increment the press count and ensure it loops between 0 and 2
            pressCount++;

            // Cycle through the positions
            if (pressCount == 1) {
                intakeRight.setPosition(0.30);
                intakeLeft.setPosition(0.70);
                intakeBack.setPosition(0.74); // Position 1 (aiming position)
            } else if (pressCount == 2) {
                intakeRight.setPosition(0.24);
                intakeLeft.setPosition(0.76);
                intakeBack.setPosition(0.85); // Position 2 (taking sample position)
            } else if (pressCount == 3) {//holding position
                intakeRight.setPosition(0.30);
                intakeLeft.setPosition(0.70);
                intakeBack.setPosition(0.72);
            }else if (pressCount == 4) {
//                if (claw_time.seconds() >= 2){
//                    sample_closed = false;
//                }
                intakeRight.setPosition(0.57); //this is the initial position
                intakeLeft.setPosition(0.43); //this = 1-intakeRight position
                intakeBack.setPosition(0.45); //this is the initial position - Position 3 (drop sample into bucket)
                pressCount = 0; // Reset to cycle back to initial position
            }
        }

        previousXState = currentXState;

        if (currentYState && !previousYState) {
            pressCount = 1;

            if (pressCount == 1){
                intakeRight.setPosition(0.30);
                intakeLeft.setPosition(0.70);
                intakeBack.setPosition(0.72);
            }
        }

        previousYState = currentYState;

        if (currentBState && !previousBState) {
            pressCount = 4;
//            claw_time.reset();

            if (pressCount == 4){
//                if (claw_time.seconds() >= 2){
//                    sample_closed = false;
//                }
                intakeRight.setPosition(0.57); //this is the initial position
                intakeLeft.setPosition(0.43); //this = 1-intakeRight position
                intakeBack.setPosition(0.45); //this is the initial position
                pressCount = 0; // Reset to cycle back to initial position
            }
        }

        previousBState = currentBState;


        if (currentRBState && !previousRBState) {
            pressCount = 5;
//            position to take close samples;

            if (pressCount == 5){
//                if (claw_time.seconds() >= 2){
//                    sample_closed = false;
//                }
                intakeRight.setPosition(0.34);
                intakeLeft.setPosition(0.66);
                intakeBack.setPosition(1);
            }
        }

        previousRBState = currentRBState;




        //bucket
        if (gamepad2.y) {
            if (!bucket_button_pressed) {
                bucket_dumped = !bucket_dumped;
            }
            bucket_button_pressed = true;
        } else bucket_button_pressed = false;

        //specimen
        if (gamepad2.a) {
            if (!specimen_button_pressed) {
                specimen_closed = !specimen_closed;
            }
            specimen_button_pressed = true;
        } else specimen_button_pressed = false;

        //hang
        if (gamepad2.dpad_right) {
            if (!hangRight_button_pressed) {
                hangRight_activated = !hangRight_activated;
            }
            hangRight_button_pressed = true;

        } else hangRight_button_pressed = false;

        if (gamepad2.dpad_right) {
            if (!hangLeft_button_pressed) {
                hangLeft_activated = !hangLeft_activated;
            }
            hangLeft_button_pressed = true;

        } else hangLeft_button_pressed = false;

    }
    public void stopVipers() {
        // Override RUN_TO_POSITION mode and stop the motors
        frontViper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backViper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontViper.setPower(0);
        backViper.setPower(0);

    }
    public void updateBooleans() {
        if (sample_closed && sample_color) {
            sample.setPosition(0.4);
        }
        else if (sample_closed || sample_color) {
            sample.setPosition(1);
            telemetry.addData("sample_color", "true");
            telemetry.addData("Color is Red", "Sample claw closes");
        }

        else{
            sample.setPosition(0.4);//this is the initial position;
        }

        if (bucket_dumped){
            bucket.setPosition(0.57);
        }
        else {
            bucket.setPosition(1);
        }

        if (specimen_closed){
            specimen.setPosition(0.68);
        }
        else {
            specimen.setPosition(0.83); //this is the initial position
        }

        if (hangRight_activated){
            hangRight.setPosition(0.7);
        }
        else {
            hangRight.setPosition(1); //this is the initial position
        }
        if (hangLeft_activated){
            hangLeft.setPosition(0.92);
        }
        else {
            hangLeft.setPosition(0.62); //this is the initial position
        }

    }

}