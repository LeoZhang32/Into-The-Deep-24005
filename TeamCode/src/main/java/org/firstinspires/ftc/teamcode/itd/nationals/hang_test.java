package org.firstinspires.ftc.teamcode.itd.nationals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class hang_test extends LinearOpMode {
    positions_and_variables positions = new positions_and_variables();
    DcMotor v_slide_front;
    DcMotor v_slide_back;
    @Override
    public void runOpMode() throws InterruptedException {
        double vert_power = positions.vert_slides_power;
        v_slide_front = hardwareMap.get(DcMotor.class, "vslidefront");
        v_slide_back = hardwareMap.get(DcMotor.class, "vslideback");
        v_slide_front.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad2.dpad_up){
                v_slide_front.setPower(vert_power);
                v_slide_back.setPower(-vert_power);
            }
            else {
                v_slide_front.setPower(0);
                v_slide_back.setPower(0);
            }
            if (gamepad2.dpad_up){
                v_slide_front.setPower(-vert_power);
                v_slide_back.setPower(vert_power);
            }
            else {
                v_slide_front.setPower(0);
                v_slide_back.setPower(0);
            }
        }
    }
}
