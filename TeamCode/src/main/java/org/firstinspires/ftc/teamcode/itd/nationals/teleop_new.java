package org.firstinspires.ftc.teamcode.itd.nationals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class teleop_new extends LinearOpMode {
    positions_and_variables positions = new positions_and_variables();

    DcMotor v_slide_front;
    DcMotor v_slide_back;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        double vert_power = positions.vert_slides_power;

        v_slide_front = hardwareMap.get(DcMotor.class, "vslidefront");
        v_slide_back = hardwareMap.get(DcMotor.class, "vslideback");
        v_slide_front.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
