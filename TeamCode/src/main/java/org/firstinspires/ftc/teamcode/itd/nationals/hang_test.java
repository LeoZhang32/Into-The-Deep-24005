package org.firstinspires.ftc.teamcode.itd.nationals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class hang_test extends LinearOpMode {
    positions_and_variables positions;
    int vert_power = positions.vert_slides_power;
    DcMotor v_slide_front;
    DcMotor v_slide_back;

    @Override
    public void runOpMode() throws InterruptedException {
        v_slide_front = hardwareMap.get(DcMotor.class, "vslidefront");
        v_slide_back = hardwareMap.get(DcMotor.class, "vslideback");
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a){
                v_slide_front.setPower(vert_power);
                v_slide_back.setPower(-vert_power);
            }
            if (gamepad1.b){
                v_slide_front.setPower(-vert_power);
                v_slide_back.setPower(vert_power);
            }
        }
    }
}
