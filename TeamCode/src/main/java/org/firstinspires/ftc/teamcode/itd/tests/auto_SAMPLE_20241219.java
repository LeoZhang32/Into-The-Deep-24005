package org.firstinspires.ftc.teamcode.itd.tests;


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
@Autonomous (name = "auto_SAMPLE_20241219")

public final class auto_SAMPLE_20241219 extends LinearOpMode {

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
                if (pos < 2200.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);

                    return false;
                }
            }
        }
        public Action liftspecimen0Up() {
            return new Liftspecimen0Up();
        }



            public class LiftuptoScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontViper.setPower(0.5);
                    backViper.setPower(0.5);
                    telemetry.addData("Position", frontViper.getCurrentPosition());
                    telemetry.addData("front Power", frontViper.getPower());
                    telemetry.addData("back Power", backViper.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos = frontViper.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2800.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action liftuptoScore (){  return new LiftuptoScore(); }






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
                if (pos < 4000.0) {
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



        public class LifttoHang implements Action {
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
                if (pos > 650.0) {
                    return true;
                } else {
                    frontViper.setPower(0);
                    backViper.setPower(0);
                    return false;
                }
            }
        }
        public Action lifttoHang(){
            return new LifttoHang();
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
                bucket.setPosition(0.57);
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
                intakeRight.setPosition(0.23);
                intakeLeft.setPosition(0.77);
                intakeBack.setPosition(0.95); // grabbing sample position
                return false;
            }
        }

        public Action extendArm() {
            return new ExtendArm();
        }


        public class RetractArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeRight.setPosition(0.54);
                intakeLeft.setPosition(0.46);
                intakeBack.setPosition(0.25);  //drop sample into bucket
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



    public class PickupSpecimen {
        private final Servo specimen;

        public PickupSpecimen(HardwareMap hardwareMap) {
            specimen = hardwareMap.get(Servo.class, "specimen");
        }


        public class CloseMClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                specimen.setPosition(0.68);
                return false;
            }
        }

        public Action closeMClaw() {
            return new CloseMClaw();
        }


        public class OpenMClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimen.setPosition(0.8);
                return false;
            }
        }

        public Action openMClaw() {
            return new OpenMClaw();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(6, 66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ScoringSample bucket = new ScoringSample(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        IntakeSample arm = new IntakeSample(hardwareMap);
        PickupSample sclaw = new PickupSample(hardwareMap);
        PickupSpecimen mclaw = new PickupSpecimen(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Actions.runBlocking(sclaw.openSClaw());
        Actions.runBlocking(arm.retractArm());
        Actions.runBlocking(mclaw.closeMClaw());

        //deposit held sample
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(36, 64))
                .strafeToLinearHeading(new Vector2d(61, 59), Math.toRadians(-135))
                .build();

        //deposit held specimen
        Action go_score_specimen_0;
        go_score_specimen_0 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(6, 34), Math.toRadians(90))
                .build();

        //go to sample 3
        Action go_to_sample_3;
        go_to_sample_3 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(24, 43), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(48, 53), Math.toRadians(-90))
                .build();

        //return to basket
        Action return_basket_3;
        return_basket_3= drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(50, 53), Math.toRadians(-112))
                .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(-135))
                .build();



        //go to  sample 2
        Action go_to_sample_2;
        go_to_sample_2 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(61, 51), Math.toRadians(-90))
                .build();

        //return to basket
        Action return_basket_2;
        return_basket_2 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(58, 53), Math.toRadians(-112))
                .strafeToLinearHeading(new Vector2d(58, 58), Math.toRadians(-135))
                .build();


        //go to sample 1
        Action go_to_sample_1;
        go_to_sample_1 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(61, 49), Math.toRadians(-62))
                .build();

        //return to basket 1
        Action return_basket_1;
        return_basket_1 = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(57, 58), Math.toRadians(-135))
                .build();


        //go to hang
        Action go_to_hang;
        go_to_hang = drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(48,48), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48,12), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(40,12), Math.toRadians(0))
                .strafeTo(new Vector2d(25,12))
                .build();




        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.1) {

            Actions.runBlocking(new SequentialAction(

                    //go score specimen 0
                            new ParallelAction(
                                    go_score_specimen_0,
                                    lift.liftspecimen0Up()
                            ),
                            new SleepAction(0.5),
                            lift.liftuptoScore(),
                            mclaw.openMClaw(),
                            new SleepAction(0.5),
                    // sample 0 cycle completes by now. sample 3 cycle starts below


                    new ParallelAction(
                        lift.liftDown(),
                        go_to_sample_3
                    ),
                    arm.extendArm(),
                    new SleepAction(1),
                    sclaw.closeSClaw(),
                    new SleepAction(0.4),

                    new ParallelAction(
                        return_basket_3,
                        new SequentialAction(
                                arm.retractArm(),
                                new SleepAction(0.8),
                                sclaw.openSClaw(),
                                new SleepAction(0.5),
                                lift.liftUp()
                        )
                    ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),
                    // sample 3 cycle completes by now. sample 2 cycle starts below


                    new ParallelAction(
                                    bucket.restoreBucket(),
                                    lift.liftDown(),
                                    go_to_sample_2,
                                    arm.extendArm()
                    ),

//                    new SleepAction(1),
                    sclaw.closeSClaw(),
                    new SleepAction(0.4),

                    new ParallelAction(
                                    return_basket_2,
                                    new SequentialAction(
                                            arm.retractArm(),
                                            new SleepAction(0.8),
                                            sclaw.openSClaw(),
                                            new SleepAction(0.5),
                                            lift.liftUp()
                                    )
                    ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),
                    // sample 2 cycle completes by now. sample 1 cycle starts below


                            new ParallelAction(
                                    bucket.restoreBucket(),
                                    lift.liftDown(),
                                    go_to_sample_1,
                                    arm.extendArm()
                            ),

//                  new SleepAction(1),
                    sclaw.closeSClaw(),
                    new SleepAction(0.4),

                    new ParallelAction(
                                    return_basket_1,
                                    new SequentialAction(
                                            arm.retractArm(),
                                            new SleepAction(0.8),
                                            sclaw.openSClaw(),
                                            new SleepAction(0.5),
                                            lift.liftUp()
                                    )
                            ),

                    bucket.dumpBucket(),
                    new SleepAction(0.7),



                    // sample 1 cycle completes by now. Go to Hang Level 1.
                    new ParallelAction(
                                bucket.restoreBucket(),
                                lift.lifttoHang(),
                                go_to_hang
                    )



                    )

            );


        }
    }

}
