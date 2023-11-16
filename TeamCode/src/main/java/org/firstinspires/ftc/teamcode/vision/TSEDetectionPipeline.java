package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class TSEDetectionPipeline extends OpenCvPipeline {
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(59, 188);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(211, 198);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(458, 188);
    static final int REGION_WIDTH = 125;
    static final int REGION_HEIGHT = 75;
    static final int REGION2_WIDTH = 225;
    static final int REGION2_HEIGHT = 25;

    static final int MIN_RED_AREA = 20;

    // Volatile since accessed by OpMode thread w/o synchronization
    private TSEPosition position = TSEPosition.LEFT;
    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */ Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
    Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    /*
     * Working variables
     */ Mat region1, region2, region3;
    int avg1, avg2, avg3;
    Scalar lowerYellow = new Scalar(18, 62, 77); // hsv
    Scalar upperYellow = new Scalar(61, 255, 255); // hsv

    Scalar lower;
    Scalar upper;


    @Override
    public void init(Mat firstFrame) {
        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
//        region1 = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        region2 = Cb.submat(new Rect(region2_pointA, region2_pointB));
//        region3 = Cb.submat(new Rect(region3_pointA, region3_pointB));
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);
        Mat mask = new Mat();
        Core.inRange(input, lowerYellow, upperYellow, mask);
        Imgproc.cvtColor(mask, input, Imgproc.COLOR_GRAY2RGB);

        region1 = new Mat(mask, new Rect(region1_pointA, region1_pointB));
        region2 = new Mat(mask, new Rect(region2_pointA, region2_pointB));
        region3 = new Mat(mask, new Rect(region3_pointA, region3_pointB));

        int sel1 = Core.countNonZero(region1);
        int sel2 = Core.countNonZero(region2);
        int sel3 = Core.countNonZero(region3);

        AutoBase.getInstance().telemetry.addData("Pixels Left", sel1);
        AutoBase.getInstance().telemetry.addData("Pixels Mid", sel2);
        AutoBase.getInstance().telemetry.addData("Pixels Right", sel3);

        if (sel1 > 1000) {
            position = TSEPosition.LEFT;
        } else if (sel2 > 1000) {
            position = TSEPosition.CENTER;
        } else if (sel3 > 1000) {
            position = TSEPosition.RIGHT;
        }


        Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

        region1.release();
        region2.release();
        region3.release();

        // Release the mask
        mask.release();

        return input;
    }


    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public TSEPosition getAnalysis() {
        return position;
    }

    public enum TSEPosition {
        LEFT, CENTER, RIGHT
    }
}