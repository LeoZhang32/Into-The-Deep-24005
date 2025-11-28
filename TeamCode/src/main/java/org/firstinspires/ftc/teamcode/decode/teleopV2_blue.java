package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop V2 Blue")
public class teleopV2_blue extends LinearOpMode {
    DecodeRobotHardwareV2 robot = new DecodeRobotHardwareV2(this);

    Boolean slowModeOn = false;
    Boolean imuReset = false;
    Boolean shooterClose = false;
    Boolean shooterFar = false;
    Boolean shooterMid = false;
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
            cycle_gamepad2.updateLB(2);

            slowModeOn = cycle_gamepad1.lbPressCount != 0;
            autoAlignOn = gamepad1.left_trigger >= 0.5 || gamepad1.right_trigger >= 0.5;
            if (autoAlignOn) {
                if (gamepad1.left_trigger >= 0.5) robot.driveToApril(true, 20, true);
                else if (gamepad1.right_trigger >= 0.5) robot.driveToApril(true, 20, false);
            }
            else {
                drive_y = -gamepad1.left_stick_y;
                drive_x = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x * 0.7;
                robot.driveRobot(drive_y, drive_x, turn, slowModeOn, imuReset);
            }

            if (cycle_gamepad2.rbPressCount == 1 && cycle_gamepad2.lbPressCount == 0){
                shooterClose = true;
                shooterFar = false;
                shooterMid = gamepad2.y;
            }
            else if (cycle_gamepad2.rbPressCount == 0 && cycle_gamepad2.lbPressCount == 1){
                shooterClose = false;
                shooterFar = true;
                shooterMid = false;
            }
            else if ((cycle_gamepad2.rbPressCount == 1 && cycle_gamepad2.lbPressCount == 1) || (cycle_gamepad2.rbPressCount == 0 && cycle_gamepad2.lbPressCount == 0)){
                cycle_gamepad2.rbPressCount = 0;
                cycle_gamepad2.lbPressCount = 0;
                shooterClose = false;
                shooterFar = false;
                shooterMid = false;
            }
            robot.intakeOuttakeAction(gamepad1.a, gamepad2.right_trigger >= 0.5, gamepad2.left_trigger >= 0.5, gamepad1.b || gamepad2.b, shooterClose, shooterFar, shooterMid);

            robot.liftAction(gamepad2.dpad_up,gamepad2.dpad_down);
        }
    }
}
