package org.firstinspires.ftc.teamcode.itd.nationals;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class intake_tele extends LinearOpMode {
    ElapsedTime transferTimer = new ElapsedTime();
    Boolean extendoIn = true;
    Servo IArmL;
    Servo IArmR;
    Servo IWrist;
    Servo IClaw;
    Servo IArmC;

    Servo OClaw;

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

        OClaw = hardwareMap.get(Servo.class, "OClaw");

        OClaw.setPosition(pos.outtake_claw_open);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
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
            if (cycle_gamepad1.aPressCount == 1){
                IClaw.setPosition(pos.intake_claw_open);
                transferTimer.reset();
                if (extendoIn){
                    if (transferTimer.milliseconds()>=200){
                        OClaw.setPosition(pos.outtake_claw_close);
                        transferTimer.reset();
                    }
                }
            }
            else {
                IClaw.setPosition(pos.intake_claw_close);
            }
        }
    }
}
