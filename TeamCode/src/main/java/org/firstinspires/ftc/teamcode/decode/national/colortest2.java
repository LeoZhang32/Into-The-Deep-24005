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
    int flickCounter = 1;
    Boolean capacityChecked = false;
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
                //if haven't checked for artifact this press, check
                if (!capacityChecked){
                    //check if each spot has artifact
                    detect1 = cSensors.checkDetected1();
                    detect2 = cSensors.checkDetected2();
                    detect3 = cSensors.checkDetected3();
                    //add detected spots to array to be shot
                    if (detect1){
                        flickOrder.add(new Flicker(flicker1, 0.65, 0.95));
                    }
                    if (detect2){
                        flickOrder.add(new Flicker(flicker2, 0.85, 0.55));
                    }
                    if (detect3){
                        flickOrder.add(new Flicker(flicker3, 0.76, 0.46));
                    }
                    //we have detected for artifacts! for next loops in press, don't check again
                    capacityChecked = true;
                }
                //if we reach the time to cycle to the next artifact, plus 1 to the counter and reset timers.
                if (nextTimer.seconds() >= 2){
                    flickerTimer.reset();
                    nextTimer.reset();
                    flickCounter += 1;
                }
                //once the counter reaches larger than the number of spots in the array.
                //this means that all artifacts have been shot.
                if (flickCounter > flickOrder.size()){
                    flickCounter = 1;
                    flickOrder.clear();
                }

                //actually move the flickers.
                if (flickerTimer.seconds() <= 1){
                    //if the timer is before time to move back, it's in score position.
                    flickOrder.get(flickCounter - 1).goScore();
                }
                //if timer is after time to move back, move back.
                else flickOrder.get(flickCounter - 1).goHome();

            }
            //once input is let go, be ready to check again, and reset everything.
            else {
                capacityChecked = false;
                flickerTimer.reset();
                nextTimer.reset();
                flickOrder.clear();
                flickCounter = 1;
                flicker1.setPosition(0.65);
                flicker2.setPosition(0.85);
                flicker3.setPosition(0.76);
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
