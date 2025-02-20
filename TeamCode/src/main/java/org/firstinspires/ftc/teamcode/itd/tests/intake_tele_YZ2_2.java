package org.firstinspires.ftc.teamcode.itd.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.itd.nationals.CycleGamepad;
import org.firstinspires.ftc.teamcode.itd.nationals.positions_and_variables;


@Disabled
@TeleOp
public class intake_tele_YZ2_2 extends LinearOpMode {
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime grabTimer = new ElapsedTime();
    boolean isTransferTimerRunning = false; // Track if timer is running
    boolean isGrabTimerRunning = false; // Track if timer is running
    Boolean extendoIn = true;
    Servo IArmL;
    Servo IArmR;
    Servo IWrist;
    Servo IClaw;
    Servo IArmC;
    Servo HSlideL;
    Servo HSlideR;

    Servo OClaw;
    Servo OArm;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

        positions_and_variables pos = new positions_and_variables();
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);

        IArmL = hardwareMap.get(Servo.class, "IArmL");
        IArmR = hardwareMap.get(Servo.class, "IArmR");
        IArmC = hardwareMap.get(Servo.class, "IArmC");
        IWrist = hardwareMap.get(Servo.class, "IWrist");
        IClaw = hardwareMap.get(Servo.class, "IClaw");
        HSlideL = hardwareMap.get(Servo.class, "HSlideL");
        HSlideR = hardwareMap.get(Servo.class, "HSlideR");

        OClaw = hardwareMap.get(Servo.class, "OClaw");
        OArm = hardwareMap.get(Servo.class, "OArm");

        OClaw.setPosition(pos.outtake_claw_open);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            cycle_gamepad1.updateX(4);
            cycle_gamepad1.updateRB(4);
            cycle_gamepad1.updateA(2);

            cycle_gamepad2.updateA(2);
            cycle_gamepad2.updateX(2);

            dashboard.sendTelemetryPacket(packet);

            //arm movements
            if (cycle_gamepad1.xPressCount == 0){
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
                cycle_gamepad1.aPressCount = 1;
                extendoIn = true;
            }
            else if (cycle_gamepad1.xPressCount == 1){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                cycle_gamepad1.aPressCount = 0;
                extendoIn = false;
            }
            else if (cycle_gamepad1.xPressCount == 2){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
                cycle_gamepad1.aPressCount = 1;
                extendoIn = false;

                if (!isGrabTimerRunning) {
                    grabTimer.reset();
                    isGrabTimerRunning = true;  // Indicate timer has started
                    telemetry.addData("Grab Timer Started", grabTimer.milliseconds());
                }
            }
            else{
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                cycle_gamepad1.aPressCount = 1;
                extendoIn = false;
            }

            //Delayed IClaw lifting
            if (isGrabTimerRunning) {
                telemetry.addData("Grab Timer Running", grabTimer.milliseconds()); // Track progress

                if (grabTimer.milliseconds() >= 300) {
                    telemetry.addData("Grab Timer Expired", grabTimer.milliseconds());

                    if (cycle_gamepad1.xPressCount == 2) {
                        cycle_gamepad1.xPressCount = 3;
                    }
                    isGrabTimerRunning = false; // Stop tracking timer once done
                }
            }

            if (gamepad1.y && !isTransferTimerRunning) {
                cycle_gamepad1.xPressCount = 1;
            }

            //wrist movements
            if (cycle_gamepad1.rbPressCount == 0){
                IWrist.setPosition(pos.intake_wrist0);
            }
            else if (cycle_gamepad1.rbPressCount == 1){
                IWrist.setPosition(pos.intake_wrist45);
            }
            else if (cycle_gamepad1.rbPressCount == 2){
                IWrist.setPosition(pos.intake_wrist90);
            }
            else{
                IWrist.setPosition(pos.intake_wrist135);
            }

            //intake claw movement
            if (!extendoIn) {
                if (cycle_gamepad1.aPressCount == 1) {
                    IClaw.setPosition(pos.intake_claw_close);
                } else {
                    IClaw.setPosition(pos.intake_claw_open);
                }
            }

            //outtake claw movement

            if (cycle_gamepad2.aPressCount == 1) {
                OClaw.setPosition(pos.outtake_claw_close);

                if (!isTransferTimerRunning) {
                    transferTimer.reset();
                    isTransferTimerRunning = true;  // Indicate timer has started
                }

            }
            else{
                OClaw.setPosition(pos.outtake_claw_open);
            }

            //Delayed IClaw opening

            if (extendoIn && isTransferTimerRunning && transferTimer.milliseconds() >= 200) {
                    IClaw.setPosition(pos.intake_claw_open);
                    cycle_gamepad1.aPressCount = 0;
                    isTransferTimerRunning = false; // Stop tracking timer once done
            }

            //outtake arm movement
            if (cycle_gamepad2.xPressCount == 0){
                OArm.setPosition(pos.outtake_arm_transfer);
            }
            else{
                OArm.setPosition(pos.outtake_arm_sample);
            }

            telemetry.addData("extendo", extendoIn);
            telemetry.addData("Grab Timer Active", isGrabTimerRunning);
            telemetry.addData("Grab Timer Time", grabTimer.milliseconds());
            telemetry.addData("Gamepad1 xPressCount", cycle_gamepad1.xPressCount);


            telemetry.addData("Gamepad1 aPressCount", cycle_gamepad1.aPressCount);
            telemetry.addData("Gamepad2 aPressCount", cycle_gamepad2.aPressCount);
            telemetry.addData("Gamepad2 xPressCount", cycle_gamepad2.xPressCount);
            telemetry.update();
        }
    }
}
