package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous (name = "auto_test5")

public final class auto_test5 extends LinearOpMode {
    DcMotor FR;
    DcMotor FL;
    DcMotor BR;
    DcMotor BL;
    Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();
    private AprilTagProcessor aprilTag;
    private static final int DESIRED_TAG_ID21 = 21;
    private static final int DESIRED_TAG_ID22 = 22;
    private static final int DESIRED_TAG_ID23 = 23;
    private AprilTagDetection desiredTag21 = null;
    private AprilTagDetection desiredTag22 = null;
    private AprilTagDetection desiredTag23 = null;
    boolean target21Found = false;
    boolean target22Found = false;
    boolean target23Found = false;
    boolean targetAligned = false;

        public class Intake {
        private final DcMotorEx intake;
        private CRServo intakeCR;

            public Intake (HardwareMap hardwareMap) {
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                intakeCR.setDirection(CRServo.Direction.REVERSE);
            }

        //IntakeRun Function
            public class IntakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(1);
                    return false;
                }
            }
            public Action IntakeRun() {
                return new IntakeRun();
            }

            //IntakeCR Function
            public class IntakeCRRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeCR.setPower(1);
                    return false;
                }
            }
            public Action IntakeCRRun() {
                return new IntakeCRRun();
            }


            //IntakeStop Function
            public class IntakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intake.setPower(0);
                    intakeCR.setPower(0);
                    return false;
                }
            }
            public Action IntakeStop() {
                return new IntakeStop();
            }


        }




        public class Outtake {
            private final DcMotorEx shooterTop;
            private final DcMotorEx shooterBottom;
            private final DcMotorEx intake;
            private Servo trigger;
            private CRServo intakeCR;
            double integralSum = 0;
            double Kp = 0.035;
            double Ki = 0;
            double Kd = 0;
            double Kf = 0.0032;
            double lastError = 0;
            ElapsedTime timer = new ElapsedTime();

            public Outtake (HardwareMap hardwareMap) {

                shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
                shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
                shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intakeCR = hardwareMap.get(CRServo.class, "intakeCR");
                trigger = hardwareMap.get(Servo.class, "trigger");

            }

            //OuttakeRun Function
            public class OuttakeRun implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    double power =  PIDControl(145, shooterTop.getVelocity(AngleUnit.DEGREES));
                    shooterTop.setPower(power);
                    shooterBottom.setPower(power);
                    return false;
                }
            }
            public Action OuttakeRun() {
                return new OuttakeRun();
            }

            public double PIDControl (double reference, double state){
                double error = reference - state;
                integralSum += error * timer.seconds();

                double derivative = (error- lastError) / timer.seconds();
                lastError = error;

                timer.reset();

                double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
                return output;
            }


            //OuttakeStop Function
            public class OuttakeStop implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    shooterTop.setPower(0);
                    shooterBottom.setPower(0);
                    return false;
                }
            }
            public Action OuttakeStop() {
                return new OuttakeStop();
            }




        }


    //trigger servo class
    public class Trigger {
        private final Servo trigger;

        public Trigger (HardwareMap hardwareMap) {
            trigger = hardwareMap.get(Servo.class, "trigger");
        }


        public class Trigger_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                trigger.setPosition(0.68);
                return false;
            }
        }

        public Action OpenTrigger() {
            return new Trigger_Open();
        }


        public class Trigger_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                trigger.setPosition(0.95);
                return false;
            }
        }

        public Action CloseTrigger() {
            return new Trigger_Closed();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {




        Pose2d beginPose = new Pose2d(-38, 52, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);
        Actions.runBlocking(trigger.CloseTrigger());
        // drivetrain motors
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(7);
        limelight.start();
        ElapsedTime LLCorrectionTimer = new ElapsedTime();
        ElapsedTime ShooterTimer = new ElapsedTime();





        //score held artifacts
        TrajectoryActionBuilder go_shoot_held_artifacts = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-15, 12), (Math.toRadians(130)));

        //go scan obelisk
        TrajectoryActionBuilder go_scan_obelisk = go_shoot_held_artifacts.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 12), (Math.toRadians(-165)));



        //After Scan, if April Tag shows PGP, then follow this sequence:
        //first PGP, then PPG



        // go to PGP
        TrajectoryActionBuilder go_from_obelisk_to_PGP = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 24), (Math.toRadians(-90)));
//
//        //go collect PGP
//        TrajectoryActionBuilder go_collect_PGP = go_from_obelisk_to_PGP.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(12, 48), Math.toRadians(-90));
//
//        //go shoot PPG
//        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-15, 12), (Math.toRadians(130)));

//        //go to PGP
//        TrajectoryActionBuilder go_from_shoot_to_PGP = go_shoot_PPG.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 24), (Math.toRadians(-90)));
//
//        //go collect PGP
//        TrajectoryActionBuilder go_collect_PGP = go_from_shoot_to_PGP.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-12, 48), Math.toRadians(-90));
//
//        //go shoot PGP
//        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-15, 12), (Math.toRadians(130)));
//
//        //LEAVE
//        TrajectoryActionBuilder go_leave = go_shoot_PGP.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(0, 48), (Math.toRadians(180)));




        //After Scan, if April Tag shows PPG, then follow this sequence:
        //first PPG, then PGP

        // go to PPG
        TrajectoryActionBuilder go_from_obelisk_to_PPG = go_scan_obelisk.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 24), (Math.toRadians(-90)));

        //go collect PPG
        TrajectoryActionBuilder go_collect_PPG = go_from_obelisk_to_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, 48), Math.toRadians(-90));

        //go shoot PPG
        TrajectoryActionBuilder go_shoot_PPG = go_collect_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, 12), (Math.toRadians(130)));

        //go to PGP
        TrajectoryActionBuilder go_from_shoot_to_PGP = go_shoot_PPG.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 24), (Math.toRadians(-90)));

        //go collect PGP
        TrajectoryActionBuilder go_collect_PGP = go_from_shoot_to_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 48), Math.toRadians(-90));

        //go shoot PGP
        TrajectoryActionBuilder go_shoot_PGP = go_collect_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, 12), (Math.toRadians(130)));

        //LEAVE
        TrajectoryActionBuilder go_leave = go_shoot_PGP.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 48), (Math.toRadians(180)));


        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <= 0.1 && !isStopRequested()) {
            Actions.runBlocking(new SequentialAction(
                    go_shoot_held_artifacts.build(),
                    outtake.OuttakeRun(),
                    new SleepAction(1.5),
                    trigger.OpenTrigger(),
                    new SleepAction(1),
                    //first artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(0.6),
                    intake.IntakeStop(),
                    new SleepAction(1.2),
                    //second artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(0.8),
                    intake.IntakeStop(),
                    new SleepAction(1.2),
                    //third artifact
                    intake.IntakeRun(),
                    intake.IntakeCRRun(),
                    new SleepAction(1),
                    intake.IntakeStop(),
                    new SleepAction(1),
                    trigger.CloseTrigger(),
                    new SleepAction(1.2),
                    go_scan_obelisk.build()
            )
            );

            while (opModeIsActive() && !isStopRequested()) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if (detection.id == DESIRED_TAG_ID21) {
                            // Yes, we want to use this tag.
                            target21Found = true;
                            desiredTag21 = detection;
                            telemetry.addData("Tag Detected", detection.id);
                            break;  // don't look any further.
                        } else if (detection.id == DESIRED_TAG_ID22){
                            // Yes, we want to use this tag.
                            target22Found = true;
                            desiredTag22 = detection;
                            telemetry.addData("Tag Detected", detection.id);
                            break;  // don't look any further.
                        } else if (detection.id == DESIRED_TAG_ID23){
                            // Yes, we want to use this tag.
                            target23Found = true;
                            desiredTag23 = detection;
                            telemetry.addData("Tag Detected", detection.id);
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

            }

            if (target21Found = true) { //GPP


            } else if  (target22Found = true) { //PGP


                Actions.runBlocking(new SequentialAction(
                                go_from_obelisk_to_PGP.build(),
                                new ParallelAction(
                                        go_collect_PGP.build(),
                                        intake.IntakeRun()
                                ),
                                new SleepAction(1.5),
                                go_shoot_PGP.build(),
                                outtake.OuttakeRun(),
                                new SleepAction(1.5),
                                trigger.OpenTrigger(),
                                new SleepAction(1),
                                //first artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.6),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //second artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(0.8),
                                intake.IntakeStop(),
                                new SleepAction(1.2),
                                //third artifact
                                intake.IntakeRun(),
                                intake.IntakeCRRun(),
                                new SleepAction(1),
                                intake.IntakeStop(),
                                new SleepAction(1),
                                trigger.CloseTrigger(),
                                new SleepAction(1.2)
                        )
                );
            } else if  (target23Found = true) { //PPG


                Actions.runBlocking(new SequentialAction(
                        go_from_obelisk_to_PPG.build(),
                        new ParallelAction(
                                go_collect_PPG.build(),
                                intake.IntakeRun()
                        ),
                        new SleepAction(1.5),
                        go_shoot_PPG.build(),
                        outtake.OuttakeRun(),
                        new SleepAction(1.5),
                        trigger.OpenTrigger(),
                        new SleepAction(1),
                        //first artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(0.6),
                        intake.IntakeStop(),
                        new SleepAction(1.2),
                        //second artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(0.8),
                        intake.IntakeStop(),
                        new SleepAction(1.2),
                        //third artifact
                        intake.IntakeRun(),
                        intake.IntakeCRRun(),
                        new SleepAction(1),
                        intake.IntakeStop(),
                        new SleepAction(1),
                        trigger.CloseTrigger(),
                        new SleepAction(1.2)
                        )
                );
            } else {

            }

            ;// Below is PPG Option (closest to Goal)




        }
    }
}


