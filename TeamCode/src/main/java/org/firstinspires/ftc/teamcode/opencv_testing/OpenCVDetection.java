package org.firstinspires.ftc.teamcode.opencv_testing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class OpenCVDetection extends OpenCvPipeline {
    public static boolean DETECT_RED = false;
    public static double MINIMUM_VALUES = 100;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_HUE = 100;
    public static double MAXIMUM_BLUE_HUE = 115;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;

    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        Left,
        Right,
        Middle
    }

    private Location location;
    /*
     * ROI is an abbreviation of Region of Interest.
     * This creates a rectangle of areas in the camera where a game element may be placed
     */
    static final Rect Left_ROI = new Rect(new Point(20, 150), new Point(60, 200));
    static final Rect Middle_ROI = new Rect(new Point(300, 150), new Point(350, 200));
    static final Rect Right_ROI = new Rect(new Point(570, 150), new Point(620, 200));

    public OpenCVDetection(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_HUE, MINIMUM_VALUES, MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HUE, MAXIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE, MINIMUM_VALUES, MINIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE, MAXIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE, MINIMUM_VALUES, MINIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE, MAXIMUM_VALUES, MAXIMUM_VALUES);

        if (!DETECT_RED) {
            //Blue value
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat);
        } else {
//Red value
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1, mat2, mat);
        }
//submat = submatrix – portion of original matrix
        Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);
        Mat middle = mat.submat(Middle_ROI);


        /*
         * We can determine the percentage of white in the image by
         * adding all of the props together and dividing by its area
         * which averages the submatrices and then dividing everything
         * by 255 (The highest value for a grayscale prop).
         *
         * Here we only take the first value of the sum result because
         * there is only one channel in a grayscale image.
         */
        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];
        double middleValue = Core.sumElems(middle).val[0];

        telemetry.addData("Left Raw Value:", leftValue);
        telemetry.addData("Right Raw Value:", rightValue);
        telemetry.addData("Middle Raw Value:", middleValue);

        left.release();
        right.release();
        middle.release();

        if (leftValue >= rightValue && leftValue >= middleValue) {
            location = Location.Left;
            telemetry.addData("Prop Location:", "Right");
        } else if (rightValue >= middleValue) {
            location = Location.Right;
            telemetry.addData("Prop Location:", "Left");
        } else {
            location = Location.Middle;
            telemetry.addData("Prop Location:", "Middle");
        }

        telemetry.update();

        /*
         * We can draw rectangles to visualize the location of the props.
         * We first convert the grayscale image back to RGB so we can
         * draw colored rectangles. Then we define two colors to differentiate
         * the prop from pixels. Finally we draw the rectangles onto the matrices.
         */
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255, 255, 255);
        Scalar propColor = new Scalar(0, 0, 255);
        /*
         * Here we used a Ternary Operator (Conditional Expression) which is
         * just a mini if statement that chooses which colors to use based
         * on the location of the prop.
         */
        Imgproc.rectangle(mat, Left_ROI, location == Location.Left ? pixelColor : propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == Location.Middle ? pixelColor : propColor);
        Imgproc.rectangle(mat, Right_ROI, location == Location.Right ? pixelColor : propColor);

        return mat;
    }
    public Location getLocation(){
        return location;
    }
}