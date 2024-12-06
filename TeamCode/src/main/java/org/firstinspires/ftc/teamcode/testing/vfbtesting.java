package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class vfbtesting extends LinearOpMode {
    Servo vfbPar0;
    Servo vfb2;
    double vfbPar0Pos = 0.447;
    double vfb2Pos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        vfbPar0 = hardwareMap.get(Servo.class,"vfbpar0");
        vfb2 = hardwareMap.get(Servo.class,"vfb2");
        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested()&&opModeIsActive()) {
            if (gamepad1.a) {
                vfb2Pos += 0.0001;
            }
            else if (gamepad1.b) vfb2Pos -= 0.0001;
            else vfb2Pos += 0;
            updateBooleans();
        }
    }
    public void updateBooleans(){
        vfb2.setPosition(vfb2Pos);
        vfbPar0.setPosition(vfbPar0Pos);
        telemetry.addData("vfbParPosition", vfbPar0Pos);
        telemetry.addData("vfb2Position", vfb2Pos);
        telemetry.update();
    }
}
