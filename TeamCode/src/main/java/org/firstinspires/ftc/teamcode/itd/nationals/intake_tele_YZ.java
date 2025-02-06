package org.firstinspires.ftc.teamcode.itd.nationals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class intake_tele_YZ extends LinearOpMode {
    ElapsedTime transferTimer = new ElapsedTime();
    Boolean extendoIn = true;
    boolean isTransferTimerRunning = false; // Track if timer is running
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
                extendoIn = true;
            }
            else if (cycle_gamepad1.xPressCount == 1){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                extendoIn = false;
            }
            else if (cycle_gamepad1.xPressCount == 2){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
                extendoIn = false;
            }
            else{
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                extendoIn = false;
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
                OClaw.setPosition(pos.outtake_claw_open);

                transferTimer.reset();
                isTransferTimerRunning = true;  // Indicate timer has started

            }
            else{
                OClaw.setPosition(pos.outtake_claw_close);
            }

            //Delayed IClaw opening
            if (extendoIn && isTransferTimerRunning && transferTimer.milliseconds() >= 300) {
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

            telemetry.addData("Gamepad1 aPressCount", cycle_gamepad1.aPressCount);
            telemetry.addData("Gamepad2 aPressCount", cycle_gamepad2.aPressCount);
            telemetry.addData("Gamepad1 xPressCount", cycle_gamepad1.xPressCount);
            telemetry.addData("Gamepad2 xPressCount", cycle_gamepad2.xPressCount);
            telemetry.update();
        }
    }
}
