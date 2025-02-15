package org.firstinspires.ftc.teamcode.itd.nationals;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.List;


@Autonomous (name = "auto_SAMPLE_test3_LimelightLeo")

public final class auto_SAMPLE_test3_LL_Leo extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    positions_and_variables pos = new positions_and_variables();

    public class Lift {
        private final DcMotorEx VSlideF;
        private final DcMotorEx VSlideB;
        private final DigitalChannel limitSwitch;

        public Lift(HardwareMap hardwareMap) {
            VSlideF = hardwareMap.get(DcMotorEx.class, "VSlideF");
            VSlideF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VSlideF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideF.setDirection(DcMotorSimple.Direction.REVERSE);

            VSlideB = hardwareMap.get(DcMotorEx.class, "VSlideB");
            VSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VSlideB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VSlideB.setDirection(DcMotorSimple.Direction.REVERSE);


            limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }


        public class Liftspecimen0Up implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    VSlideF.setPower(0.5);
                    VSlideB.setPower(0.5);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();

                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
                if (pos_VSF < 2050.0) {
                    return true;
                } else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);

                    return false;
                }
            }
        }
        public Action liftspecimen0Up() {
            return new Liftspecimen0Up();
        }





        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {

                    VSlideF.setPower(1);
                    VSlideB.setPower(1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
                if (pos_VSF < 2750 && pos_VSB > -2750) {
                    return true;
                } else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
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
                    VSlideF.setPower(-1);
                    VSlideB.setPower(-1);
                    telemetry.addData("front Position", VSlideF.getCurrentPosition());
                    telemetry.addData("back Position", VSlideB.getCurrentPosition());
                    telemetry.addData("front Power", VSlideF.getPower());
                    telemetry.addData("back Power", VSlideB.getPower());
                    telemetry.update();
                    initialized = true;
                }

                double pos_VSF = VSlideF.getCurrentPosition();
                packet.put("liftPosF", pos_VSF);
                double pos_VSB = VSlideB.getCurrentPosition();
                packet.put("liftPosB", pos_VSB);
//                boolean switchPressed = limitSwitch.getState();
                if (pos_VSF > 50 && pos_VSB < -50 && limitSwitch.getState()) {
                    return true;
                }
                else {
                    VSlideF.setPower(0);
                    VSlideB.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }

    }


    //OClaw servo class
    public class OuttakeClaw {
        private final Servo OClaw;

        public OuttakeClaw (HardwareMap hardwareMap) {
            OClaw = hardwareMap.get(Servo.class, "OClaw");
        }


        public class OClaw_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                OClaw.setPosition(pos.outtake_claw_open);
                return false;
            }
        }

        public Action OpenOClaw() {
            return new OClaw_Open();
        }


        public class OClaw_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OClaw.setPosition(pos.outtake_claw_close);
                return false;
            }
        }

        public Action CloseOClaw() {
            return new OClaw_Closed();
        }

    }



    public class OuttakeArm {
        private final Servo OArm;

        public OuttakeArm (HardwareMap hardwareMap) {
            OArm = hardwareMap.get(Servo.class, "OArm");
        }


        public class OArm_Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                OArm.setPosition(pos.outtake_arm_sample);
                return false;
            }
        }

        public Action LiftOArm() {
            return new OArm_Up();
        }


        public class OArm_Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                OArm.setPosition(pos.outtake_arm_transfer);
                return false;
            }
        }

        public Action LowerOArm() {
            return new OArm_Down();
        }

    }



    //intake servos class
    public class Intake {
        private final Servo IArmL;
        private final Servo IArmR;
        private final Servo IClaw;
        private final Servo IArmC;
        private final Servo HSlideL;
        private final Servo HSlideR;

        public Intake (HardwareMap hardwareMap) {
            IArmL = hardwareMap.get(Servo.class, "IArmL");
            IArmR = hardwareMap.get(Servo.class, "IArmR");
            IArmC = hardwareMap.get(Servo.class, "IArmC");
            IClaw = hardwareMap.get(Servo.class, "IClaw");
            HSlideL = hardwareMap.get(Servo.class, "HSlideL");
            HSlideR = hardwareMap.get(Servo.class, "HSlideR");
        }


        public class Intake_Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HSlideL.setPosition(pos.hslide_trans);
                HSlideR.setPosition(1-pos.hslide_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
                IClaw.setPosition(pos.intake_claw_close); //sample transfer position
                return false;
            }
        }

        public Action SettoTrasfer () {  return new Intake_Transfer();
        }


        public class Intake_AfterTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HSlideL.setPosition(pos.hslide_after_trans);
                HSlideR.setPosition(1-pos.hslide_after_trans);
                IArmL.setPosition(pos.intake_arm_lift);
                IArmR.setPosition(1-pos.intake_arm_lift);
                IArmC.setPosition(pos.intake_coax_lift);
                IClaw.setPosition(pos.intake_claw_open); //after transfer position
                return false;
            }
        }

        public Action SettoAfterTrasfer () {
            return new Intake_AfterTransfer();
        }




        public class Intake_Aim implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_aim);
                IArmR.setPosition(1-pos.intake_arm_aim);
                IArmC.setPosition(pos.intake_coax_aim);
                IClaw.setPosition(pos.intake_claw_open); // aiming position
                return false;
            }
        }

        public Action SettoAim () {
            return new Intake_Aim();
        }


        public class Intake_Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_aim);
                HSlideR.setPosition(1-pos.hslide_aim);
                IArmL.setPosition(pos.intake_arm_grab);
                IArmR.setPosition(1-pos.intake_arm_grab);
                IArmC.setPosition(pos.intake_coax_grab);
                IClaw.setPosition(pos.intake_claw_close); // grabbing sample
                return false;
            }
        }

        public Action SettoGrab() {

            return new Intake_Grab();
        }


        public class Intake_AfterGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                HSlideL.setPosition(pos.hslide_after_trans);
                HSlideR.setPosition(1-pos.hslide_after_trans);
                IArmL.setPosition(pos.intake_arm_trans);
                IArmR.setPosition(1-pos.intake_arm_trans);
                IArmC.setPosition(pos.intake_coax_trans);
                IClaw.setPosition(pos.intake_claw_close); // after grabbing sample
                return false;
            }
        }

        public Action SettoAfterGrab() {

            return new Intake_AfterGrab();
        }
    }






    //IntakeClaw servo class
    public class IntakeClaw {
        private final Servo IClaw;

        public IntakeClaw (HardwareMap hardwareMap) {
            IClaw = hardwareMap.get(Servo.class, "IClaw");
        }


        public class IClaw_Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IClaw.setPosition(pos.intake_claw_open);
                return false;
            }
        }

        public Action OpenIClaw() {
            return new IClaw_Open();
        }


        public class IClaw_Closed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                IClaw.setPosition(pos.intake_claw_close);
                return false;
            }
        }

        public Action CloseIClaw() {
            return new IClaw_Closed();
        }

    }




    //sample claw wrist servo class
    public class IntakeWrist {
        private final Servo IWrist;

        public IntakeWrist (HardwareMap hardwareMap) {
            IWrist = hardwareMap.get(Servo.class, "IWrist");
        }


        public class Wrist0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist0);
                return false;
            }
        }

        public Action SettoWrist0() {
            return new Wrist0();
        }


        public class Wrist45 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist45);
                return false;
            }
        }

        public Action SettoWrist45() {
            return new Wrist45();
        }


        public class Wrist90 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist90);
                return false;
            }
        }

        public Action SettoWrist90() {
            return new Wrist90();
        }


        public class Wrist135 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                IWrist.setPosition(pos.intake_wrist135);
                return false;
            }
        }

        public Action SettoWrist135() {
            return new Wrist135();
        }


    }
    public class Limelight {
        private final Limelight3A limelight;
        private final PinpointDrive drive;
        public Limelight (PinpointDrive drive, HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            this.drive = drive;
        }
        public class LimelightCorrection implements Action {

            private boolean initialized = false;

            private static final double STRAFE_COEFF = 0.005;  // Tune this constant
            private static final double FORWARD_COEFF = 0.005;  // Tune this constant
            private static final double THRESHOLD = 2.0;  // Tolerance for offset

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double tx = 0;
                double ty = 0;
                if (!initialized) {
                    limelight.setPollRateHz(100);
                    telemetry.setMsTransmissionInterval(11);
                    limelight.pipelineSwitch(0);
                    limelight.start();
                    initialized = true;
                }

                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        tx = result.getTx();
                        ty = result.getTy();
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    }
                }

                double strafePower = ty * STRAFE_COEFF;
                double forwardPower = tx * FORWARD_COEFF;

                packet.put("tx", tx);
                packet.put("ty", ty);
                packet.put("strafePower", strafePower);
                packet.put("forwardPower", forwardPower);

                Vector2d correctionVector = new Vector2d(strafePower, forwardPower);
                drive.setDrivePowers(new PoseVelocity2d(correctionVector, 0.0));

                if (Math.abs(tx) < THRESHOLD && Math.abs(ty) < THRESHOLD) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false;
                }
                else {
                    return true;
                }
            }
        }
        public Action limelightCorrection(){return new LimelightCorrection();}
    }







    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(40, 63.5, (Math.toRadians(-90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Lift lift = new Lift(hardwareMap);
        OuttakeClaw oclaw = new OuttakeClaw(hardwareMap);
        OuttakeArm oarm = new OuttakeArm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        IntakeClaw iclaw = new IntakeClaw(hardwareMap);
        IntakeWrist wrist = new IntakeWrist(hardwareMap);
        Limelight limelight = new Limelight(drive, hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Actions.runBlocking(oclaw.CloseOClaw());
        Actions.runBlocking(oarm.LowerOArm());
        Actions.runBlocking(intake.SettoAfterTrasfer());
        Actions.runBlocking(iclaw.OpenIClaw());
        Actions.runBlocking(wrist.SettoWrist0());
        DigitalChannel limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //score held sample
        TrajectoryActionBuilder go_score_sample_0 = drive.actionBuilder(beginPose)

                .strafeToSplineHeading(new Vector2d(58, 58), (Math.toRadians(-135)));

        //go to sample 3
        TrajectoryActionBuilder go_to_sample_3 = go_score_sample_0.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(24, 43), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(49, 49), (Math.toRadians(-88)));

        //return to basket
        TrajectoryActionBuilder return_basket_3 = go_to_sample_3.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)));


        //go to sample 2
        TrajectoryActionBuilder go_to_sample_2 = return_basket_3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58, 48), Math.toRadians(-90));


        //return to basket
        TrajectoryActionBuilder return_basket_2 = go_to_sample_2.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(57, 57), (Math.toRadians(-135)));


        //go to sample 1
        TrajectoryActionBuilder go_to_sample_1 = return_basket_2.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(43, 27.5), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(47.5, 27.5), (Math.toRadians(0)));


        //return to basket 1
        TrajectoryActionBuilder return_basket_1 = go_to_sample_1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(43, 25), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(57, 58), (Math.toRadians(-135)));



        //go to hang
        TrajectoryActionBuilder go_to_hang = return_basket_1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(35,12), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(20,12), (Math.toRadians(0)));





        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 0.01) {


            boolean switchPressed = !limitSwitch.getState(); // Inverted to show "pressed" as true

            // Display the state on the telemetry
            if (switchPressed) {
                telemetry.addData("Limit Switch", "Pressed");
            } else {
                telemetry.addData("Limit Switch", "Not Pressed");
            }

            telemetry.update();




            Actions.runBlocking(new SequentialAction(
                            new ParallelAction(
                                    go_score_sample_0.build(),
                                    lift.liftUp()
                            ),
                            oarm.LiftOArm(),
                            new SleepAction(0.7),
                            oclaw.OpenOClaw(),
                            new SleepAction(0.4),
                            // sample 0 cycle completes by now. sample 3 cycle starts below

                            new ParallelAction(
                                    go_to_sample_3.build(),
                                    oarm.LowerOArm(),
                                    lift.liftDown()
                            ),

                            intake.SettoAim(),
                            new SleepAction(0.6),
                            limelight.limelightCorrection()

                    )

            );
            drive.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    intake.SettoGrab()
            ));


        }
    }

}
