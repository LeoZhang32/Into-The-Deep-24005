package org.firstinspires.ftc.teamcode.itd.auto;


import static org.firstinspires.ftc.teamcode.rr.MecanumDrive.normalizeAngle;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;


// hang specimen 0 from top down
@Autonomous (name = "auto_SPECIMEN_1_Preload")

public final class auto_SPECIMEN_1_Preload extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    double wristPosition = 0.23;

    public class Lift {
        private final DcMotorEx frontViper;
        private final DcMotorEx backViper;

        public Lift(HardwareMap hardwareMap) {
            frontViper = hardwareMap.get(DcMotorEx.class, "frontViper");
            frontViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontViper.setDirection(DcMotorSimple.Direction.REVERSE);
            backViper = hardwareMap.get(DcMotorEx.class, "backViper");
            backViper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backViper.setDirection(DcMotorSimple.Direction.FORWARD);
             }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    frontViper.setPower(1);
                    backViper.setPower(1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();

                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2250.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);

                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }




        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontViper.setPower(-1);
                    backViper.setPower(-1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 50.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }





        public class LiftuptoMiddle implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    frontViper.setPower(1);
                    backViper.setPower(1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();

                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 410.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);

                    return false;
                }
            }
        }
        public Action liftuptoMiddle () {
            return new LiftuptoMiddle();
        }





        public class LiftdowntoMiddle implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontViper.setPower(-1);
                    backViper.setPower(-1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 360.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action liftdowntoMiddle(){
            return new LiftdowntoMiddle();
        }



        public class LiftdowntoScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontViper.setPower(-0.5);
                    backViper.setPower(-0.5);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1700.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action liftdowntoScore (){
            return new LiftdowntoScore();
        }





        public class Liftspecimen0Up implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    frontViper.setPower(1);
                    backViper.setPower(1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();

                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2050.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);

                    return false;
                }
            }
        }
        public Action Liftspecimen0Up() {
            return new Liftspecimen0Up();
        }


        public class Liftspecimen2Up implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontViper.setPower(1);
                    backViper.setPower(1);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2400.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action Liftspecimen2Up (){
            return new Liftspecimen2Up();
        }





    }


    //bucket servo class
    public class ScoringSample {
        private final Servo bucket;

        public ScoringSample(HardwareMap hardwareMap) {
            bucket = hardwareMap.get(Servo.class, "bucket");
        }


        public class DumpBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bucket.setPosition(0.57); //0.57
                return false;
            }
        }

        public Action dumpBucket() {
            return new DumpBucket();
        }


        public class RestoreBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(1);
                return false;
            }
        }

        public Action restoreBucket() {
            return new RestoreBucket();
        }

    }



    //intake servos class
    public class IntakeSample {
        private final Servo intakeRight;
        private final Servo intakeLeft;
        private final Servo intakeBack;

        public IntakeSample(HardwareMap hardwareMap) {
            intakeRight = hardwareMap.get(Servo.class, "intakeRight");
            intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
            intakeBack = hardwareMap.get(Servo.class, "intakeBack");
        }




        public class AimArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeRight.setPosition(0.31);
                intakeLeft.setPosition(0.69);
                intakeBack.setPosition(0.94); // aiming position
                return false;
            }
        }


        public Action aimArm() {

            return new AimArm();
        }



        public class ExtendArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeRight.setPosition(0.28);
                intakeLeft.setPosition(0.72);
                intakeBack.setPosition(0.94); // grabbing sample position (a bit lower than teleop)
                return false;
            }
        }

        public Action extendArm() {
            return new ExtendArm();
        }




        public class RetractArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRight.setPosition(0.61);
                intakeLeft.setPosition(0.39);
                intakeBack.setPosition(0.3); //drop sample into bucket
                return false;
            }
        }

        public Action retractArm() {  return new RetractArm();
        }

    }






    //sample claw servo class
    public class PickupSample {
        private final Servo sample;

        public PickupSample(HardwareMap hardwareMap) {
            sample = hardwareMap.get(Servo.class, "sample");
        }


        public class CloseSClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sample.setPosition(0.68);
                return false;
            }
        }

        public Action closeSClaw() {
            return new CloseSClaw();
        }


        public class OpenSClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sample.setPosition(1);
                return false;
            }
        }

        public Action openSClaw() {
            return new OpenSClaw();
        }

    }



        //sample claw wrist servo class
        public class SampleWrist {
            private final Servo sampleWrist;

            public SampleWrist(HardwareMap hardwareMap) {
                sampleWrist = hardwareMap.get(Servo.class, "sampleWrist");
            }


            public class Wrist1 implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sampleWrist.setPosition(wristPosition);
                    return false;
                }
            }

            public Action Wrist1() {
                return new Wrist1();
            }


            public class Wrist2 implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sampleWrist.setPosition(wristPosition + 0.15);
                    return false;
                }
            }

            public Action Wrist2() {
                return new Wrist2();
            }


            public class Wrist3 implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sampleWrist.setPosition(wristPosition + 0.3);
                    return false;
                }
            }

            public Action Wrist3() {
                return new Wrist3();
            }



            public class Wrist4 implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sampleWrist.setPosition(wristPosition - 0.15);
                    return false;
                }
            }

            public Action Wrist4() {
                return new Wrist4();
            }


        }





    public class PickupSpecimen {
        private final Servo specimen;

        public PickupSpecimen(HardwareMap hardwareMap) {
            specimen = hardwareMap.get(Servo.class, "specimen");
        }


        public class CloseMClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimen.setPosition(0.66);
                return false;
            }
        }

        public Action closeMClaw() {
            return new CloseMClaw();
        }


        public class OpenMClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimen.setPosition(0.83);
                return false;
            }
        }

        public Action openMClaw() {
            return new OpenMClaw();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(-8.3, 66, normalizeAngle(Math.toRadians(90)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ScoringSample bucket = new ScoringSample(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        IntakeSample arm = new IntakeSample(hardwareMap);
        PickupSample sclaw = new PickupSample(hardwareMap);
        PickupSpecimen mclaw = new PickupSpecimen(hardwareMap);
        SampleWrist wrist = new SampleWrist(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Actions.runBlocking(sclaw.openSClaw());
        Actions.runBlocking(arm.retractArm());
        Actions.runBlocking(mclaw.closeMClaw());
        Actions.runBlocking(wrist.Wrist1());


        //score held specimen
        TrajectoryActionBuilder go_score_specimen_0 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-8.3, 35), normalizeAngle(Math.toRadians(90)));


//        //go get specimen 1 (if alliance member does not use it in auto)
//        TrajectoryActionBuilder go_get_specimen_1 = go_score_specimen_0.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-37, 60), normalizeAngle(Math.toRadians(-90)))
//                .strafeToLinearHeading(new Vector2d(-37, 66), normalizeAngle(Math.toRadians(-90)));
//
//
//        //go score specimen 1 (if alliance member does not use it in auto)
//        TrajectoryActionBuilder go_score_specimen_1 = go_get_specimen_1.endTrajectory().fresh()
//                .strafeToSplineHeading(new Vector2d(-8, 40), normalizeAngle(Math.toRadians(90)))
//                .strafeToSplineHeading(new Vector2d(-8, 35), normalizeAngle(Math.toRadians(90)));



        //go pick up sample 4
        TrajectoryActionBuilder go_pickup_sample_4 = go_score_specimen_0.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(-50, 46.5), normalizeAngle(Math.toRadians(-90.0000001)))
                ;

        //go dump sample 4
        TrajectoryActionBuilder go_dump_sample_4 = go_pickup_sample_4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61.5, 45.5), normalizeAngle(Math.toRadians(-90.0000001)))
                ;

//        //go pick up sample 5
//        TrajectoryActionBuilder go_pickup_sample_5 = go_dump_sample_4.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-60.5, 48), normalizeAngle(Math.toRadians(-90)))
//                ;
//
//        //go dump sample 5
//        TrajectoryActionBuilder go_dump_sample_5 = go_pickup_sample_5.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-60.5, 49), normalizeAngle(Math.toRadians(-90)))
//                ;


        //go get specimen 2
        TrajectoryActionBuilder go_get_specimen_2 = go_dump_sample_4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-37, 60), normalizeAngle(Math.toRadians(-90)))
                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(-37, 65), normalizeAngle(Math.toRadians(-90)));

        //go score specimen 2
        TrajectoryActionBuilder go_score_specimen_2 = go_get_specimen_2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-6.5, 40), normalizeAngle(Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-6.5, 34), normalizeAngle(Math.toRadians(90)));


        //there may not be enough time to get and score specimen 3
        //go get specimen 3
        TrajectoryActionBuilder go_get_specimen_3 = go_score_specimen_2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-43, 58), normalizeAngle(Math.toRadians(-90.0000001)))
                .strafeToSplineHeading(new Vector2d(-43, 64), normalizeAngle(Math.toRadians(-88)));

        //go score specimen 3
        TrajectoryActionBuilder go_score_specimen_3 = go_get_specimen_3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0, 40), normalizeAngle(Math.toRadians(89)))
                .strafeToSplineHeading(new Vector2d(0, 33), normalizeAngle(Math.toRadians(89)));


        //go park
        TrajectoryActionBuilder go_park = go_score_specimen_2.endTrajectory().fresh()

                .strafeToSplineHeading(new Vector2d(-37, 56), normalizeAngle(Math.toRadians(160)));



        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.01) {

            Actions.runBlocking(new SequentialAction(

                            //go score specimen 0
                            new ParallelAction(
                                    go_score_specimen_0.build(),
                                    lift.Liftspecimen0Up()
                            ),
                            new SleepAction(0.3),
                            lift.liftdowntoScore(),
                            mclaw.openMClaw(),
                            new SleepAction(0.3),


                            //go pick up samples 4
                            new ParallelAction(
                                    go_pickup_sample_4.build(),
                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            lift.liftDown(),
                                            new SleepAction(0.6),
                                            arm.aimArm()
                                    )
                            ),
                            arm.extendArm(),
                            new SleepAction(0.3),
                            sclaw.closeSClaw(),
                            new SleepAction(0.3),

                            //go dump sample 4
                            new ParallelAction(
                                 go_dump_sample_4.build(),
                                 new SequentialAction(
                                    arm.retractArm(),
                                    new SleepAction(0.9),
                                    sclaw.openSClaw(),
                                    new SleepAction(0.4),
                                    lift.liftuptoMiddle()
                                 )
                            ),

                            new ParallelAction(
                                    bucket.dumpBucket(),
                                    arm.aimArm()
                            ),

                            new SleepAction(0.9),

                            //go pick up sample 5
                            arm.extendArm(),


                            new SleepAction(0.3),
                            new ParallelAction(
                                    bucket.restoreBucket(),
                                    sclaw.closeSClaw()
                            ),
                            new SleepAction(0.3),


                            //go dump sample 5
                            new ParallelAction(
//                                 go_dump_sample_5.build(),
                                    new SequentialAction(
                                            new ParallelAction(
                                                    arm.retractArm(),
                                                    new SequentialAction(
                                                            new SleepAction(0.2),
                                                            lift.liftDown()
                                                    )
                                            ),
                                            new SleepAction(0.9),
                                            sclaw.openSClaw(),
                                            new SleepAction(0.4),
                                            lift.liftuptoMiddle()
                                    )
                            ),

                            bucket.dumpBucket(),
                            new SleepAction(0.3),



                            //go get specimen 2
                            new ParallelAction(
                                    go_get_specimen_2.build(),
                                    lift.liftdowntoMiddle(),
                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            bucket.restoreBucket()
                                    )

                            ),
                            mclaw.closeMClaw(),
                            new SleepAction(0.3),

                            //go score specimen 2
                            new ParallelAction(
                                    lift.liftUp(),
                                    new SequentialAction(
                                            new SleepAction(0.1),
                                            go_score_specimen_2.build()
                                    )
                            ),
                            new SleepAction(0.3),
                            lift.liftdowntoScore(),
                            mclaw.openMClaw(),
                            new SleepAction(0.3),


                            //there may not be enough time to get and score specimen 3, so code not used.

                    //go get specimen 3
                    new ParallelAction(
                            go_get_specimen_3.build(),
                            new SequentialAction(
                                    new SleepAction(0.3),
                                    lift.liftdowntoMiddle()
                            )
                    ),
                    mclaw.closeMClaw(),
                    new SleepAction(0.3),

                    //go score specimen 3
                    new ParallelAction(
                            lift.liftUp(),
                            new SequentialAction(
                                    new SleepAction(0.1),
                                    go_score_specimen_3.build()
                            )
                    ),
                    new SleepAction(0.3),
                    lift.liftdowntoScore(),
                    mclaw.openMClaw(),
                    new SleepAction(0.3),


                            //always go park
                    new ParallelAction(
                                    go_park.build(),
                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            lift.liftDown()
                                    ),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            arm.aimArm()
                                    )
                    ),
                    arm.extendArm()


                    )
            );


        }
    }

}
