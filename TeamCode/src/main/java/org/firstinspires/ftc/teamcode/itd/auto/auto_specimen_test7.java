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
@Autonomous (name = "auto_specimen_test7")

public final class auto_specimen_test7 extends LinearOpMode {

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
                if (pos < 310.0) {
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
                if (pos > 290.0) {
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
                if (pos < 2200.0) {
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
        public Action liftuptoScore (){
            return new LiftuptoScore();
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


        Pose2d beginPose = new Pose2d(0, 66, Math.toRadians(90));
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

        //deposit held specimen
        Action go_score_specimen_0;
        go_score_specimen_0 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(0, 35.5))
                .build();



        //go get specimen 1
        Action go_get_specimen_1;
        go_get_specimen_1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-40, 50))
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-40, 67))
                .build();


        //go score specimen 1
        Action go_score_specimen_1;
        go_score_specimen_1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-40, 50))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-6, 42), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-9, 35), Math.toRadians(90))
//                .turn(Math.toRadians(180))
//                .strafeTo(new Vector2d(-9, 50))
//                .strafeTo(new Vector2d(-9, 35))
                .build();



        //push 2 samples and get specimen 1
        Action push_2_samples_and_get_specimen_1;
        push_2_samples_and_get_specimen_1 = drive.actionBuilder(drive.pose)

                .strafeTo(new Vector2d(-38, 36))
//                .strafeTo(new Vector2d(-38, 36))
                .strafeTo(new Vector2d(-40, 14.5))
                .strafeTo(new Vector2d(-48, 14.5))
                .strafeTo(new Vector2d(-48, 54))
                .strafeTo(new Vector2d(-48, 14.5))
                .strafeTo(new Vector2d(-56, 14.5))
                .strafeTo(new Vector2d(-56, 58))

//                .strafeTo(new Vector2d(-56, 46))
//                .strafeTo(new Vector2d(-48, 46))
//                .turn(Math.toRadians(182))
//                .strafeTo(new Vector2d(-48, 65))
//                .strafeToLinearHeading(new Vector2d(-48,46), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(-48,67), Math.toRadians(-90))
                .build();


        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.1) {

            Actions.runBlocking(new SequentialAction(

                    //go score specimen 0
                    new ParallelAction(
                            go_score_specimen_0,
                            lift.Liftspecimen0Up()
                    ),
                    new SleepAction(0.5),
                    lift.liftuptoScore(),
                    mclaw.openMClaw(),
                    new SleepAction(0.5),

                    //go get specimen 1
                    new ParallelAction(
                            go_get_specimen_1,
                            new SequentialAction(
                                    new SleepAction(1),
                                    lift.liftdowntoMiddle()
                            )
                    ),
                    mclaw.closeMClaw(),
                    new SleepAction(0.5),

                    //go score specimen 1
                    new ParallelAction(
                                    lift.liftUp(),
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            go_score_specimen_1
                                    )
                    ),
                    new SleepAction(1.5),
                    lift.liftdowntoScore(),
                    mclaw.openMClaw(),
                    new SleepAction(0.5),




                    //push_2_samples_and_get_specimen_1
                    new ParallelAction(
                            push_2_samples_and_get_specimen_1,
                            new SequentialAction(
                                    new SleepAction(1),
                                    lift.liftdowntoMiddle()
                            )
                    )



                            /*,


                    new SleepAction(1),


                    mclaw.closeMClaw(),


                    new SleepAction(0.5),




                    //go get specimen 2
                    new ParallelAction(
                            go_get_specimen_2,
                            new SequentialAction(
                                     new SleepAction(1),
                                     lift.liftdowntoMiddle()
                            )
                    ),
                    mclaw.closeMClaw(),


                    //go score specimen 2
                    new ParallelAction(
                            lift.liftUp(),
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    go_score_specimen_2
                            )
                    ),
                    new SleepAction(0.5),
                    lift.liftdowntoScore(),
                    mclaw.openMClaw(),
                    new SleepAction(0.5),

                    //go get specimen 3
                    new ParallelAction(
                            go_get_specimen_3,
                            new SequentialAction(
                                    new SleepAction(1),
                                    lift.liftdowntoMiddle()
                            )

                    )
                    */

                    )
            );


        }
    }

}
