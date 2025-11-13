package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="Teleop Test3")
public class teleop_test3 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    Boolean slowModeOn = false;
    Boolean imuReset = false;
    Boolean shooterOn = false;
    Boolean intakeForward = false;
    Boolean intakeBackward = false;
    Boolean intakeServoForward = false;
    Boolean autoAlignOn = false;

    @Override
    public void runOpMode() {
        CycleGamepad cycle_gamepad1 = new CycleGamepad(gamepad1);
        CycleGamepad cycle_gamepad2 = new CycleGamepad(gamepad2);
        double drive_y;
        double drive_x;
        double turn;
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            imuReset = gamepad1.start;
            cycle_gamepad1.updateLB(2);
            cycle_gamepad2.updateRB(2);

            slowModeOn = cycle_gamepad1.lbPressCount != 0;
            autoAlignOn = gamepad1.left_trigger >= 0.5;
            if (autoAlignOn) {
                robot.driveToApril(true, 24);
            }
            else {
                drive_y = -gamepad1.left_stick_y;
                drive_x = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x * 0.7;
                robot.driveRobot(drive_y, drive_x, turn, slowModeOn, imuReset);
            }

            shooterOn = cycle_gamepad2.rbPressCount != 0;
            intakeForward = gamepad1.a || gamepad2.a;
            intakeBackward = gamepad1.b || gamepad2.b;
            intakeServoForward = gamepad2.right_trigger >=0.5;
//            robot.intakeOuttakeAction(intakeForward, intakeServoForward,intakeBackward,shooterOn);

            robot.liftAction(gamepad2.dpad_up,gamepad2.dpad_down);
        }
    }
}
