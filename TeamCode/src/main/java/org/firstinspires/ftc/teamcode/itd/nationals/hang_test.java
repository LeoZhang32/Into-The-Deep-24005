package org.firstinspires.ftc.teamcode.itd.nationals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class hang_test extends LinearOpMode {
    Boolean VS_manual_running = false;
    Boolean VS_auto_up_button_pressed = false;
    Boolean VS_auto_up = false;
    Boolean VS_auto_down_button_pressed = false;
    Boolean VS_auto_down = false;
    Boolean VS_specscore_button_pressed = false;
    Boolean VS_specscore = false;

    DcMotor VSlideF;
    DcMotor VSlideB;

    Servo OArm;
    DigitalChannel limitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        positions_and_variables pos = new positions_and_variables();
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);
        VSlideF = hardwareMap.get(DcMotor.class, "VSlideF");
        VSlideB = hardwareMap.get(DcMotor.class, "VSlideB");
        OArm = hardwareMap.get(Servo.class, "OArm");
        VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);
        VSlideB.setDirection(DcMotorSimple.Direction.REVERSE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        OArm.setPosition(pos.outtake_arm_transfer);
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            VSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cycle_gamepad2.updateLB(3);
            cycle_gamepad2.updateX(2);
            if (gamepad2.y) {
                if (!VS_auto_up_button_pressed) {
                    VS_auto_up = !VS_auto_up;
                }
                VS_auto_up_button_pressed = true;
            } else VS_auto_up_button_pressed = false;
            if (gamepad2.b) {
                if (!VS_auto_down_button_pressed) {
                    VS_auto_down = !VS_auto_down;
                }
                VS_auto_down_button_pressed = true;
            } else VS_auto_down_button_pressed = false;

            if (cycle_gamepad2.lbPressCount == 2){
                OArm.setPosition(pos.outtake_arm_specimen);
            }
            else if (cycle_gamepad2.lbPressCount == 1||cycle_gamepad2.xPressCount == 1){
                OArm.setPosition(pos.outtake_arm_sample);
            }
            else if (cycle_gamepad2.lbPressCount == 0||cycle_gamepad2.xPressCount == 0){
                OArm.setPosition(pos.outtake_arm_transfer);
            }

            if (gamepad2.right_bumper) {
                if (!VS_specscore_button_pressed) {
                    VS_specscore = !VS_specscore;
                }
                VS_specscore_button_pressed = true;
            } else VS_specscore_button_pressed = false;

            if (!limitSwitch.getState() && !VS_auto_up && !VS_auto_down && !VS_manual_running) {
                //if pressed
                VSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            V_SLIDES:
            if (gamepad2.dpad_up) {
                VS_manual_running = true;
                if (VSlideF.getCurrentPosition() > 2850 || VSlideB.getCurrentPosition() > 2850) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    telemetry.addData("viper slides", "over limit");
                    telemetry.update();
                    break V_SLIDES;
                }
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(1);
                VSlideB.setPower(1);

            } else if (gamepad2.dpad_down) {
                VS_manual_running = true;
                if (!limitSwitch.getState()) {
                    //if limit switch is pressed and dpad down
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    telemetry.addData("viper slides", "stopped");
                    telemetry.update();
                    break V_SLIDES;
                }
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(-1);
                VSlideB.setPower(-1);


            } else if (VS_auto_up) { //viper slide auto actions

                VS_manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(1);
                VSlideB.setPower(1);


                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        VS_auto_up = !VS_auto_up;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach top");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }

                if (VSlideF.getCurrentPosition() > 2750 || VSlideB.getCurrentPosition() > 2750) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    VS_auto_up = !VS_auto_up;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }

            } else if (VS_auto_down) { //viper slide auto action down

                VS_manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(-1);
                VSlideB.setPower(-1);


                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        VS_auto_down = !VS_auto_down;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach bottom");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }


                if (VSlideF.getCurrentPosition() < 30 || VSlideB.getCurrentPosition() < 30) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    VS_auto_down = !VS_auto_down;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }


            } else if (VS_specscore) { //viper slide auto action down score

                VS_manual_running = false;
                VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                VSlideF.setPower(0.5);
                VSlideB.setPower(0.5);

                if ((VSlideF.isBusy()) || (VSlideB.isBusy()) || !isStopRequested()) {

                    // Check for an emergency stop condition
                    if (gamepad2.start) { // **ADDED: Use right bumper for emergency stop**
                        // **ADDED: Stop the motors immediately**
                        VSlideF.setPower(0);
                        VSlideB.setPower(0);
                        VS_specscore = !VS_specscore;
//                            break; // **ADDED: Exit the loop on emergency stop**

                    }

                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting to reach bottom");
                    telemetry.addData("VSlideF power", VSlideF.getPower());
                    telemetry.addData("VSlideB power", VSlideB.getPower());
                    telemetry.addData("VSlideF position", VSlideF.getCurrentPosition());
                    telemetry.addData("VSlideF position", VSlideB.getCurrentPosition());
                    telemetry.addData("is at target", !VSlideF.isBusy() && !VSlideB.isBusy());
                    telemetry.update();
                }


                if (VSlideF.getCurrentPosition() > 130 || VSlideB.getCurrentPosition() > 130) {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    VS_specscore = !VS_specscore;
                    telemetry.addData("Status", "position reached");
                    telemetry.update();
                }

            }
            else {
                VS_manual_running = false;
                VSlideF.setPower(0);
                VSlideB.setPower(0);
            }
        }
    }
}
