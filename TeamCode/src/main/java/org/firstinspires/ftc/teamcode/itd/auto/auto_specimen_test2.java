package org.firstinspires.ftc.teamcode.itd.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;


@Disabled
@Autonomous (name = "auto_specimen_test2")

public final class auto_specimen_test2 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

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
                if (pos < 2000.0) {
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
                if (pos < 710.0) {
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
                if (pos > 690.0) {
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
                bucket.setPosition(0.6);
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


        public class ExtendArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeRight.setPosition(0.3);
                intakeLeft.setPosition(0.7);
                intakeBack.setPosition(0.8);
                return false;
            }
        }

        public Action extendArm() {
            return new ExtendArm();
        }


        public class RetractArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRight.setPosition(0.57);
                intakeLeft.setPosition(0.43);
                intakeBack.setPosition(0.45);
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
                sample.setPosition(1);
                return false;
            }
        }

        public Action closeSClaw() {
            return new CloseSClaw();
        }


        public class OpenSClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sample.setPosition(0.4);
                return false;
            }
        }

        public Action openSClaw() {
            return new OpenSClaw();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(0, 66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ScoringSample bucket = new ScoringSample(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        IntakeSample arm = new IntakeSample(hardwareMap);
        PickupSample sclaw = new PickupSample(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Actions.runBlocking(sclaw.openSClaw());
        Actions.runBlocking(arm.retractArm());

        //go hang specimen 0
        Action go_hang_specimen_0;
        go_hang_specimen_0 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(0, 38))
                .strafeTo(new Vector2d(0, 35))
                .build();


        //go to sample 1 middle point
        Action go_to_sample_1_mp;
        go_to_sample_1_mp = drive.actionBuilder(drive.pose)

//                .strafeToLinearHeading(new Vector2d(-12, 48), Math.toRadians(135))
                .splineTo(new Vector2d(-24,60), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-36, 55), Math.toRadians(225))
                .splineTo(new Vector2d(-50,51), Math.toRadians(-90))
                .build();


        //go to sample 1
        Action go_to_sample_1;
        go_to_sample_1 = drive.actionBuilder(drive.pose)

//                .strafeToLinearHeading(new Vector2d(-50, 51), Math.toRadians(-90))
                .splineTo(new Vector2d(-50,47),Math.toRadians(-90))
                .build();


        //return to observation zone 1
        Action return_observation_1;
        return_observation_1= drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-60,60),Math.toRadians(-90))
                .build();


        //get sample 2
        Action go_to_sample_2;
        go_to_sample_2 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-60, 51))
                .strafeTo(new Vector2d(-60,47))
                .build();

        //return to observation zone 2
        Action return_observation_2;
        return_observation_2= drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-60,60))
                .build();


        //get sample 3
        Action go_to_sample_3;
        go_to_sample_3 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-62, 47.5), Math.toRadians(-115))
                .strafeToLinearHeading(new Vector2d(-62.5,45), Math.toRadians(-113))
                .build();


        //return to observation zone 3
        Action return_observation_3;
        return_observation_3= drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-60,60), Math.toRadians(-90))
                .build();


        //go to specimen 1
        Action go_to_specimen_1;
        go_to_specimen_1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-40,63))
                .strafeTo(new Vector2d(-40,66))
                .build();


        //go hang specimen 1
        Action go_hang_specimen_1;
        go_hang_specimen_1 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-12,54), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(0,38), Math.toRadians(90))
                .strafeTo(new Vector2d(0,35))
                .build();


        //go to specimen 2
        Action go_to_specimen_2;
        go_to_specimen_2 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-30,42), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(-40,63), Math.toRadians(-90))
                .strafeTo(new Vector2d(-40,66))
                .build();

        //go hang specimen 2
        Action go_hang_specimen_2;
        go_hang_specimen_2 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-12,54), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(0,38), Math.toRadians(90))
                .strafeTo(new Vector2d(0,35))
                .build();

        //go to specimen 3
        Action go_to_specimen_3;
        go_to_specimen_3 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-30,42), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(-40,63), Math.toRadians(-90))
                .strafeTo(new Vector2d(-40,66))
                .build();

        //go hang specimen 3
        Action go_hang_specimen_3;
        go_hang_specimen_3 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(-12,54), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(0,38), Math.toRadians(90))
                .strafeTo(new Vector2d(0,35))
                .build();


        //park
        Action go_park;
        go_park = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-40,65))
                .build();



        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.1) {

            Actions.runBlocking(new SequentialAction(

                    new ParallelAction(

                        go_hang_specimen_0,
                        lift.liftUp()
                        ),

                    //MClaw actions to hang specimen 0

                    new SleepAction(0.7),


                    // specimen 0 cycle completes by now. sample 1 cycle starts below

                    new ParallelAction(
                        go_to_sample_1_mp,
                        lift.liftDown()
                    ),


                    new ParallelAction(
                        go_to_sample_1,
                        arm.extendArm()
                    ),

                    sclaw.closeSClaw(),
                    new SleepAction(0.7),

                    new ParallelAction(
                        return_observation_1,
                        new SequentialAction(
                                arm.retractArm(),
                                new SleepAction(0.8),
                                sclaw.openSClaw(),
                                new SleepAction(0.5),
                                lift.liftuptoMiddle()
                        )
                    ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),
                    bucket.restoreBucket(),
                    // sample 1 cycle completes by now. sample 2 cycle starts below


                    new ParallelAction(

                               go_to_sample_2,
                               lift.liftDown(),
                               arm.extendArm()
                    ),

                    sclaw.closeSClaw(),
                    new SleepAction(0.7),

                    new ParallelAction(
                                    return_observation_2,
                                    new SequentialAction(
                                            arm.retractArm(),
                                            new SleepAction(0.8),
                                            sclaw.openSClaw(),
                                            new SleepAction(0.5),
                                            lift.liftuptoMiddle()
                                    )
                    ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),
                    bucket.restoreBucket(),
                    // sample 2 cycle completes by now. sample 3 cycle starts below


                            new ParallelAction(

                                    go_to_sample_3,
                                    lift.liftDown(),
                                    arm.extendArm()
                            ),


                    sclaw.closeSClaw(),
                    new SleepAction(0.7),

                    new ParallelAction(
                                    return_observation_3,
                                    new SequentialAction(
                                            arm.retractArm(),
                                            new SleepAction(0.8),
                                            sclaw.openSClaw(),
                                            new SleepAction(0.5),
                                            lift.liftuptoMiddle()
                                    )
                            ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),
                    bucket.restoreBucket(),


                    // sample 3 cycle completes by now. Specimen 1 cycle starts below.

                    go_to_specimen_1,


                    //MClaw actions to pick up specimen 1
                    new SleepAction(1),


                    new ParallelAction(

                            go_hang_specimen_1,
                            lift.liftUp()

                    ),


                    //MClaw actions to hang specimen 1
                    new SleepAction(1),

                    new ParallelAction(

                            go_to_specimen_2,
                            lift.liftdowntoMiddle()

                    ),
                    // Specimen 1 cycle completes by now. Specimen 2 cycle starts below.

                    //MClaw actions to pick up specimen 2
                    new SleepAction(1),

                    new ParallelAction(

                            go_hang_specimen_2,
                            lift.liftUp()

                    ),


                    //MClaw actions to hang specimen 2
                    new SleepAction(1),

                    new ParallelAction(

                            go_to_specimen_3,
                            lift.liftdowntoMiddle()
                    ),
                    // Specimen 2 cycle completes by now. Specimen 3 cycle starts below.

                    //MClaw actions to pick up specimen 3
                    new SleepAction(1),

                    new ParallelAction(

                            go_hang_specimen_3,
                            lift.liftUp()
                    ),


                    //MClaw actions to hang specimen 3
                    new SleepAction(1),

                    new ParallelAction(

                            go_park,
                            lift.liftDown()
                    )

                    )

            );


        }
    }

}
