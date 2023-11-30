package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TSEDetectionPipelineRightRed extends OpenCvPipeline {
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(99 - 80, 188);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(251 - 80, 153 + 25);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(498 - 80, 188);
    static final int REGION_WIDTH = 125;
    static final int REGION_HEIGHT = 75 + 40;
    static final int REGION2_WIDTH = 225;
    static final int REGION2_HEIGHT = 60 + 40;

    static final int MIN_RED_AREA = 20;

    // Volatile since accessed by OpMode thread w/o synchronization
    private AutoConstants.TSEPosition position = AutoConstants.TSEPosition.LEFT;
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
    Scalar lowerYellow = new Scalar(50, 0, 0); // rgb
    Scalar upperYellow = new Scalar(170, 40, 40); // rgb

    Scalar lower;
    Scalar upper;


    @Override
    public void init(Mat firstFrame) {
    }


    @Override
    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
            Core.inRange(input, lowerYellow, upperYellow, input);

            region1 = new Mat(input, new Rect(region1_pointA, region1_pointB));
            region2 = new Mat(input, new Rect(region2_pointA, region2_pointB));
            region3 = new Mat(input, new Rect(region3_pointA, region3_pointB));

            int sel1 = Core.countNonZero(region1);
            int sel2 = Core.countNonZero(region2);
            int sel3 = Core.countNonZero(region3);

            AutoBase.getInstance().telemetry.addData("Pixels Left", sel1);
            AutoBase.getInstance().telemetry.addData("Pixels Mid", sel2);
            AutoBase.getInstance().telemetry.addData("Pixels Right", sel3);

            if (sel1 > 1000) {
                position = AutoConstants.TSEPosition.LEFT;
            } else if (sel2 > 500) {
                position = AutoConstants.TSEPosition.CENTER;
            } else if (sel3 > 1000) {
                position = AutoConstants.TSEPosition.RIGHT;
            }


            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

            region1.release();
            region2.release();
            region3.release();

            try {
                sleep(100);
            } catch (InterruptedException e) {
            }
        } catch (Exception e) {}

        return input;
    }


    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public AutoConstants.TSEPosition getAnalysis() {
        return position;
    }

}