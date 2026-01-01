package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class timertest extends LinearOpMode {
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    int flickerCount = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        nextTimer.reset();
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a){
                if (nextTimer.seconds() >= 2){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickerCount += 1;
                }
                if (flickerCount == 4){
                    flickerCount = 1;
                }
                if (flickerCount == 1){
                    if (flickerTimer.seconds() <= 1){
                        flicker1.setPosition(0.95);
                    }
                    else flicker1.setPosition(0.65);
                }
                else if (flickerCount == 2){
                    if (flickerTimer.seconds() <= 1){
                        flicker2.setPosition(0.55);
                    }
                    else flicker2.setPosition(0.85);
                }
                else if (flickerCount == 3){
                    if (flickerTimer.seconds() <= 1){
                        flicker3.setPosition(0.46);
                    }
                    else flicker3.setPosition(0.76);
                }
            }
            else {
                flicker1.setPosition(0.65);
                flicker2.setPosition(0.85);
                flicker3.setPosition(0.76);

            }
            telemetry.addData("nexttimer:", nextTimer.seconds());
            telemetry.update();
        }
    }
}
