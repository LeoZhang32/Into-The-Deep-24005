package org.firstinspires.ftc.teamcode.decode.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class colortest1 extends LinearOpMode {
    color_sensor_hardware cSensors = new color_sensor_hardware();
    String detectedColor;
    Enum a;
    Enum b;
    @Override
    public void runOpMode() throws InterruptedException {
        cSensors.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
//            a = cSensors.get2ADetectedColors();
//            b = cSensors.get2BDetectedColors();
            if (a == color_sensor_hardware.DetectedColor.PURPLE ||b == color_sensor_hardware.DetectedColor.PURPLE){
                detectedColor = "PURPLE";
            }
            else if(a == color_sensor_hardware.DetectedColor.GREEN ||b == color_sensor_hardware.DetectedColor.GREEN){
                detectedColor = "GREEN";
            }
            else {
                detectedColor = "UNKNOWN";
            }
            telemetry.addData("COLOR",detectedColor);
            telemetry.update();
        }
    }
}
