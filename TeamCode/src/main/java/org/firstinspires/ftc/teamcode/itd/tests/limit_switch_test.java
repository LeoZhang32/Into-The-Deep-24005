package org.firstinspires.ftc.teamcode.itd.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@Disabled
@TeleOp
public class limit_switch_test extends LinearOpMode {
    DigitalChannel limitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannel.class,"limitSwitch");
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            if (limitSwitch.getState()){
                telemetry.addData("state","true");
            }
            else {
                telemetry.addData("state","false");
            }
            telemetry.update();
        }
    }
}
