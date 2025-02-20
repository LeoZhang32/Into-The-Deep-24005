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
public class iclawtele extends LinearOpMode {
    ElapsedTime transferTimer = new ElapsedTime();
    Boolean extendoIn = true;
    Boolean isTransferTimerRunning = false;
    Boolean timerrunning = true;// Track if timer is running
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
            if (cycle_gamepad1.aPressCount == 1){
                IClaw.setPosition(pos.intake_claw_open);
                telemetry.addData("pause completed", "false");

                boolean timerpause = pause(transferTimer, 200);
                if (timerpause){
                    OClaw.setPosition(pos.outtake_claw_close);
                    telemetry.addData("pause completed", "true");
                }

            }
            //closed
            else {
                IClaw.setPosition(pos.intake_claw_close);
            }
            if (timerrunning){
                transferTimer.reset();
                timerrunning = false;
            }
            telemetry.update();
        }
    }
    public boolean pause(ElapsedTime timer, double waitTime) {
        return timer.milliseconds() >= waitTime;
    }
}
