package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
public class colortest2 extends LinearOpMode {
    color_sensor_hardware cSensors = new color_sensor_hardware();
    String detectedColor;
    Boolean detect1;
    Boolean detect2;
    Boolean detect3;
    ElapsedTime nextTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    ArrayList<Flicker> flickOrder = new ArrayList<>();
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    int flickCounter = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        flicker1 = hardwareMap.servo.get("flicker1");
        flicker2 = hardwareMap.servo.get("flicker2");
        flicker3 = hardwareMap.servo.get("flicker3");
        cSensors.init(hardwareMap);
        nextTimer.reset();
        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a){
                detect1 = cSensors.checkDetected1();
                detect2 = cSensors.checkDetected2();
                detect3 = cSensors.checkDetected3();
                if (detect1){
                    flickOrder.add(new Flicker(flicker1, 0.65, 0.95));
                }
                if (detect2){
                    flickOrder.add(new Flicker(flicker2, 0.85, 0.55));
                }
                if (detect3){
                    flickOrder.add(new Flicker(flicker3, 0.76, 0.46));
                }
                if (nextTimer.seconds()>=2){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickCounter += 1;
                }
                if (flickCounter > flickOrder.size()){
                    flickCounter = 0;
                    flickOrder.clear();
                }
            }

        }
    }

    class Flicker {
        Servo servo;
        double home, score;
        Flicker(Servo servo, double home, double score){
            this.servo = servo;
            this.home = home;
            this.score = score;
        }
        void goHome(){
            servo.setPosition(home);
        }
        void goScore(){
            servo.setPosition(score);
        }
    }
}
