package org.firstinspires.ftc.teamcode.decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot Hardware Class Teleop Test1")
public class teleop_hardwareclasstest1 extends LinearOpMode {
    DecodeRobotHardware robot = new DecodeRobotHardware(this);

    Boolean slowModeOn = false;
    Boolean imuReset = false;
    Boolean outtakeOn = false;

    @Override
    public void runOpMode() {
        CycleGamepad cycle_gampepad1 = new CycleGamepad(gamepad1);
        double drive_y = 0;
        double drive_x = 0;
        double turn = 0;
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            imuReset = gamepad1.start;
            cycle_gampepad1.updateLB(2);
            cycle_gampepad1.updateX(6);
            cycle_gampepad1.updateRB(2);

            outtakeOn = cycle_gampepad1.rbPressCount == 1;

            drive_y = -gamepad1.left_stick_y;
            drive_x = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x * 0.7;
            slowModeOn = cycle_gampepad1.lbPressCount != 0;

            robot.driveRobot(drive_y, drive_x, turn, slowModeOn, imuReset);
//            robot.shooterCycle(gamepad1.right_bumper);
            robot.intakeOuttakeAction(gamepad1.a, gamepad1.b, outtakeOn);
        }
    }
}
