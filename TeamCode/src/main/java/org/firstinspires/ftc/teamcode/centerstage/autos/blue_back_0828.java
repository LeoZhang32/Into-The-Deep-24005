package org.firstinspires.ftc.teamcode.centerstage.autos;

import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.findContours;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Blue Back Aug 28")
public final class blue_back_0828 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double cX = 0;
    double cY = 0;
    double width = 0;
    double area;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.54;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    private int r = 0;    //  蓝0        红100
    private int rs = 10;   //  蓝10       红124
    private int g = 43;     //
    private int gs = 255;   //
    private int b = 46;     //
    private int bs = 255;   //
    private int detection_num = 0;//自治道具位置

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
        Pose2d beginPose = new Pose2d(-36, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        IntakeDrop intake_drop = new IntakeDrop(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        initOpenCV();
        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        trajectoryAction3 = drive.actionBuilder(drive.pose)
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
                .splineTo(new Vector2d(56,60), Math.toRadians(0))
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(24)
                .waitSeconds(1)
                .afterDisp(16, intake_drop.dropIntake())
                .lineToY(40)
                .waitSeconds(0.5)
                .turn(Math.toRadians(90))
                .afterDisp(10,intake_drop.liftIntake())
                .lineToX(36)
                .turn(Math.toRadians(90))
                .lineToY(60)
                .turn(Math.toRadians(-90))
                .lineToX(55)
                .build();
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(36)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .lineToX(-22)
                .waitSeconds(0.5)
                .afterDisp(16,intake_drop.dropIntake())
                .lineToX(-38)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .afterDisp(10, intake_drop.liftIntake())
                .lineToY(12)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .lineToX(36)
                .turn(Math.toRadians(90))
                .lineToY(60)
                .turn(Math.toRadians(-90))
                .lineToX(60)
                .build();
        Actions.runBlocking(intake_drop.liftIntake());
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= 1.5){
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Area in Pixels:", String.valueOf(area));
            if(300 < cX && cX < 750 && area > 20000){
                detection_num = 2;
            }
            else if(750 <= cX && area > 20000){
                detection_num = 3;
            }
//            else if (cX <= 300  && area > 10000){
//                detection_num = 1;
//            }
            else {
                detection_num = 1;
            }
            telemetry.addData("Position:", "%4d",detection_num);
            telemetry.update();
        }
        controlHubCam.stopStreaming();
        if (detection_num == 3) {
            Actions.runBlocking(
                trajectoryAction3
            );
        }
        else if (detection_num == 2){
            Actions.runBlocking(
                trajectoryAction2
            );
        }
        else if (detection_num == 1){
            Actions.runBlocking(
                trajectoryAction1
            );
        }
        else{
            telemetry.addData("error:", "nothing detected");
            telemetry.update();
        }
    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new RedBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class RedBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions 对帧进行预处理以检测黄色区域
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                area = contourArea(largestContour);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);
                //Display area
                String areaLabel = "Area: " + (int) area + "pixels";
                Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 100), Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0), 3);
                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 0), Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0), 3);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 50), Imgproc.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0), 3);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(r, g, b);
            Scalar upperYellow = new Scalar(rs, gs, bs);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }



}
