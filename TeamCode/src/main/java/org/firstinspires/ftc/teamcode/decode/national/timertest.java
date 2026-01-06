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

    public static double home1 = 0.96;
    public static double home2 = 0.03;
    public static double home3 = 0.175;
    public static double score1 = 0.66;
    public static double score2 = 0.33;
    public static double score3 = 0.475;
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
                if (nextTimer.seconds() >= 0.4){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickerCount += 1;
                }
                if (flickerCount == 4){
                    flickerCount = 1;
                }
                if (flickerCount == 1){
                    if (flickerTimer.seconds() <= 0.25){
                        flicker1.setPosition(score1);
                    }
                    else flicker1.setPosition(home1);
                }
                else if (flickerCount == 2){
                    if (flickerTimer.seconds() <= 0.25){
                        flicker2.setPosition(score2);
                    }
                    else flicker2.setPosition(home2);
                }
                else if (flickerCount == 3){
                    if (flickerTimer.seconds() <= 0.25){
                        flicker3.setPosition(score3);
                    }
                    else flicker3.setPosition(home3);
                }
            }
            else {
                flicker1.setPosition(home1);
                flicker2.setPosition(home2);
                flicker3.setPosition(home3);

            }
            telemetry.addData("nexttimer:", nextTimer.seconds());
            telemetry.update();
        }
    }
}
