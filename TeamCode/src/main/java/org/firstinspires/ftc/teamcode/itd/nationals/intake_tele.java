package org.firstinspires.ftc.teamcode.itd.nationals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class intake_tele extends LinearOpMode {
    Servo IArmL;
    Servo IArmR;
    Servo IWrist;
    Servo IClaw;
    Servo IArmC;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
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

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            cycle_gamepad1.updateX(4);
            cycle_gamepad1.updateLB(4);
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
            if (cycle_gamepad1.lbPressCount == 0){
                IWrist.setPosition(pos.intake_wrist0);
            }
            else if (cycle_gamepad1.lbPressCount == 1){
                IWrist.setPosition(pos.intake_wrist1);
            }
            else if (cycle_gamepad1.lbPressCount == 2){
                IWrist.setPosition(pos.intake_wrist2);
            }
            else{
                IWrist.setPosition(pos.intake_wrist3);
            }
            //claw movement
            if (cycle_gamepad1.aPressCount == 0){
                IClaw.setPosition(pos.intake_claw_open);
            }
        }
    }
}
