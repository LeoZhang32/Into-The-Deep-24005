package org.firstinspires.ftc.teamcode.itd.nationals;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class iclawtele extends LinearOpMode {
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
            //initial
            if (cycle_gamepad1.aPressCount == 0){
                IClaw.setPosition(pos.intake_claw_open);
            }
            //closed
            else {
                IClaw.setPosition(pos.intake_claw_close);
            }

            //arm movements
            if (cycle_gamepad1.xPressCount == 0){
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
            }
            else if (cycle_gamepad1.xPressCount == 1){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
            }
            else if (cycle_gamepad1.xPressCount == 2){
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
            }
            else{
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
            }
            if (cycle_gamepad2.aPressCount == 1){
                OClaw.setPosition(pos.outtake_claw_close);
                if (!isTransferTimerRunning && cycle_gamepad1.aPressCount == 1 && cycle_gamepad1.xPressCount == 0){
                    transferTimer.reset();
                    isTransferTimerRunning = true;
                }
            }
            else {
                OClaw.setPosition(pos.outtake_claw_open);
            }
            if (transferTimer.milliseconds() >= 300 && isTransferTimerRunning){
                cycle_gamepad1.aPressCount = 0;
                transferTimer.reset();
                isTransferTimerRunning = false;
            }
        }
    }
}
