package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.itd.nationals.CycleGamepad;

@Disabled
@TeleOp
public class cycletest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            cycle_gamepad1.updateX(2);
            telemetry.addData("cycle number", cycle_gamepad1.xPressCount);
            telemetry.update();
        }
    }
}
