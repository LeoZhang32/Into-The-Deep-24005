package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class flickers_hardware {
    Servo flicker1;
    Servo flicker2;
    Servo flicker3;
    public void init (HardwareMap hwMap){
        flicker1 = hwMap.get(Servo.class, "flicker1");
        flicker2 = hwMap.get(Servo.class, "flicker2");
        flicker3 = hwMap.get(Servo.class, "flicker3");
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
