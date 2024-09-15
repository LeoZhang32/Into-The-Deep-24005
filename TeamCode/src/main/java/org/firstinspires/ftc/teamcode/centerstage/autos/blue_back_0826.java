package org.firstinspires.ftc.teamcode.centerstage.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.TuningOpModes;

@Autonomous (name = "Blue Back Aug 26")
public final class blue_back_0826 extends LinearOpMode {
    //intake servo class
    public class IntakeDrop {
        private Servo intake_drop;

        public IntakeDrop(HardwareMap hardwareMap) {
            intake_drop = hardwareMap.get(Servo.class, "intake_drop");
        }


        public class DropIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake_drop.setPosition(0.9);
                return false;
            }
        }

        public Action dropIntake() {
            return new DropIntake();
        }

        public class LiftIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_drop.setPosition(0.4);
                return false;
            }
        }

        public Action liftIntake() {
            return new LiftIntake();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeDrop intake_drop = new IntakeDrop(hardwareMap);

//        Pose2d beginPose = new Pose2d(-36, 63, Math.toRadians(0));
        Pose2d beginPose = new Pose2d(-36, 63, Math.toRadians(-90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Actions.runBlocking(intake_drop.liftIntake());
            waitForStart();
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //move to purple scoring position
                            .lineToY(36)
                            .waitSeconds(1)
                            //after two seconds, drop the intake to release purple.
                            .turn(Math.toRadians(-90))
                            .lineToX(-44)
                            .waitSeconds(0.5)
                            .afterDisp(14, intake_drop.dropIntake())
                            .lineToX(-30)
                            .waitSeconds(0.5)
                            //intake should drop ~2 secs into wait
//                            .waitSeconds(3)
                            //after 15 inches of moving (next line),
                            .afterDisp(15, intake_drop.liftIntake())
                            //yellow scoring position
                            .lineToXConstantHeading(36)
                            //parking
                            .turn(Math.toRadians(-90))
                            .splineTo(new Vector2d(56, 60), Math.toRadians(0))
                            .build());
        }
    }
}
