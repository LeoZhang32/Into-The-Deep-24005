package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.CycleGamepad;

@TeleOp
public class shooter_2 extends LinearOpMode {
    DcMotorEx intake;
    DcMotorEx outtake;
    Servo trigger;

    ElapsedTime triggerTimer = new ElapsedTime();
    boolean isTriggerTimerRunning = false; // Track if timer is running
    boolean velocityValid = false;
    double shooterVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);
        intake = hardwareMap.get(DcMotorEx.class, "abc");
        outtake = hardwareMap.get(DcMotorEx.class, "def");
        trigger = hardwareMap.get(Servo.class, "trigger");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad1.x){
                outtake.setPower(0.7);
                telemetry.addData("velocity",shooterVelocity);

            }
            else outtake.setPower(0);
            shooterVelocity = outtake.getVelocity(AngleUnit.DEGREES);
            velocityValid = shooterVelocity >= 120;
            if (velocityValid) trigger.setPosition(0.75);
            else trigger.setPosition(0.95);
            telemetry.update();
            if (gamepad1.a){
                intake.setPower(1);
            }
            else intake.setPower(0);
        }
    }
}