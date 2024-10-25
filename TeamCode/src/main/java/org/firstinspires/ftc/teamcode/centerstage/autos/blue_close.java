package org.firstinspires.ftc.teamcode.centerstage.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.opencv_testing.OpenCVDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class blue_close extends LinearOpMode {
    /*
     * If program has a build folder error try clearing the build
     */
    OpenCvWebcam camera;

    public class Claw {
        public Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }
    }
    public class IntakeDrop {
        public Servo intake_drop;

        public IntakeDrop(HardwareMap hardwareMap) {
            intake_drop = hardwareMap.get(Servo.class, "intake_drop");
        }
        public class PullUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_drop.setPosition(0);
                return false;
            }
        }
        public Action pullUp() {
            return new PullUp();
        }
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_drop.setPosition(0.9);
                return false;
            }
        }

        public Action dropIntake() {
            return new Drop();
        }

        
    }

    @Override
    public void runOpMode(){
        Action trajectoryActionChosen1;
        Action trajectoryActionChosen2;
        Claw claw = new Claw(hardwareMap);
        IntakeDrop intake_drop = new IntakeDrop(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(-90)));
        Servo claw_wrist;
        Servo claw_elbow;
        Servo drone;
        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action secondTrajectory1;
        Action secondTrajectory2;
        Action secondTrajectory3;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(40)
                .turn(Math.toRadians(-90))
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(40)
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToY(40)
                .turn(Math.toRadians(90))
                .build();
        secondTrajectory1 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(180))
                .build();
        secondTrajectory2 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .build();
        secondTrajectory3 = drive.actionBuilder(drive.pose)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        OpenCVDetection detector = new OpenCVDetection(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000);

        // Timeout for obtaining permission is configurable. Set |
        /*
         * Below is an example of a lambda expression which is in simply an anonymous function.
         * Since we are only executing one statement we are able to remove the curly braces and semicolon
         * making it look much cleaner.
         * Note that this is a feature strictly for SDK 8+, if Java 7 is being used use this code instead.
         * To change preferences press command and ; to open up preference window.
         *
         * Lambda Expression *
         * camera.openCameraDeviceAsync(() -> camera.startStreaming (320,240, OpenCvCameraRotation. UPRIGHT));
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 60);

        waitForStart();
        if (isStopRequested()) return;

        switch (detector.getLocation()) {
            case Left:
                // ...
                trajectoryActionChosen1 = trajectoryAction1;
                //trajectoryActionChosen2 =
                break;
            case Right:
                // ...
                trajectoryActionChosen1 = trajectoryAction2;
                break;
            case Middle:
                // ...
                trajectoryActionChosen1 = trajectoryAction3;
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + detector.getLocation());
        }
        Actions.runBlocking(
                new SequentialAction(
                        intake_drop.pullUp(),
                        claw.closeClaw(),
                        trajectoryActionChosen1,
                        intake_drop.dropIntake()
                )
        );
        camera.stopStreaming();
    }
}