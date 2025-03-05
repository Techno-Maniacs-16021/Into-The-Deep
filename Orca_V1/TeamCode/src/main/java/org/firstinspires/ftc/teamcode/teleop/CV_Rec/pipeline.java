import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public static class SkystoneDeterminationPipeline extends OpenCvPipeline
{
    /*
     * An enum to define the 9 regions of stuff
     */
    private String Alliance = "Blue";
    public changeAlliance(String Color) {
        Alliance = Color;
    }
    public enum DeepPosition
    {
        LeftTop,
        MiddleTop,
        RightTop,
        LeftMiddle,
        MiddleMiddle,//lol
        RightMiddle,
        LeftBottom,
        MiddleBottom,
        RightBottom
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    /*
     * The core values which define the location and size of the sample regions
     */
    // Row #1 (top)
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,   0);   // LeftTop
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(107, 0);   // MiddleTop
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(214, 0);   // RightTop

    // Row #2 (middle)
    static final Point REGION4_TOPLEFT_ANCHOR_POINT = new Point(0,   80);  // LeftMiddle
    static final Point REGION5_TOPLEFT_ANCHOR_POINT = new Point(107, 80);  // MiddleMiddle
    static final Point REGION6_TOPLEFT_ANCHOR_POINT = new Point(214, 80);  // RightMiddle

    // Row #3 (bottom)
    static final Point REGION7_TOPLEFT_ANCHOR_POINT = new Point(0,   160); // LeftBottom
    static final Point REGION8_TOPLEFT_ANCHOR_POINT = new Point(107, 160); // MiddleBottom
    static final Point REGION9_TOPLEFT_ANCHOR_POINT = new Point(214, 160); // RightBottom
    static final int REGION_HEIGHT = 80;

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
     *Currenty areas
     *
     * 1 2 3
     * 4 5 6
     * 7 8 9
     */
// Region 1 (LeftTop)
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + 107, // width for column 1
            REGION1_TOPLEFT_ANCHOR_POINT.y + 80); // row height

    // Region 2 (MiddleTop)
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + 107, // width for column 2
            REGION2_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 3 (RightTop)
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + 106, // width for column 3
            REGION3_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 4 (LeftMiddle)
    Point region4_pointA = new Point(
            REGION4_TOPLEFT_ANCHOR_POINT.x,
            REGION4_TOPLEFT_ANCHOR_POINT.y);
    Point region4_pointB = new Point(
            REGION4_TOPLEFT_ANCHOR_POINT.x + 107,
            REGION4_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 5 (MiddleMiddle)
    Point region5_pointA = new Point(
            REGION5_TOPLEFT_ANCHOR_POINT.x,
            REGION5_TOPLEFT_ANCHOR_POINT.y);
    Point region5_pointB = new Point(
            REGION5_TOPLEFT_ANCHOR_POINT.x + 107,
            REGION5_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 6 (RightMiddle)
    Point region6_pointA = new Point(
            REGION6_TOPLEFT_ANCHOR_POINT.x,
            REGION6_TOPLEFT_ANCHOR_POINT.y);
    Point region6_pointB = new Point(
            REGION6_TOPLEFT_ANCHOR_POINT.x + 106,
            REGION6_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 7 (LeftBottom)
    Point region7_pointA = new Point(
            REGION7_TOPLEFT_ANCHOR_POINT.x,
            REGION7_TOPLEFT_ANCHOR_POINT.y);
    Point region7_pointB = new Point(
            REGION7_TOPLEFT_ANCHOR_POINT.x + 107,
            REGION7_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 8 (MiddleBottom)
    Point region8_pointA = new Point(
            REGION8_TOPLEFT_ANCHOR_POINT.x,
            REGION8_TOPLEFT_ANCHOR_POINT.y);
    Point region8_pointB = new Point(
            REGION8_TOPLEFT_ANCHOR_POINT.x + 107,
            REGION8_TOPLEFT_ANCHOR_POINT.y + 80);

    // Region 9 (RightBottom)
    Point region9_pointA = new Point(
            REGION9_TOPLEFT_ANCHOR_POINT.x,
            REGION9_TOPLEFT_ANCHOR_POINT.y);
    Point region9_pointB = new Point(
            REGION9_TOPLEFT_ANCHOR_POINT.x + 106,
            REGION9_TOPLEFT_ANCHOR_POINT.y + 80);


    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat region4_Cb, region5_Cb, region6_Cb;
    Mat region7_Cb, region8_Cb, region9_Cb;

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    int avg1, avg2, avg3;
    int avg4, avg5, avg6;
    int avg7, avg8, avg9;


    private volatile DeepPosition position = DeepPosition.LEFT;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        region4_Cb = Cb.submat(new Rect(region4_pointA, region4_pointB));
        region5_Cb = Cb.submat(new Rect(region5_pointA, region5_pointB));
        region6_Cb = Cb.submat(new Rect(region6_pointA, region6_pointB));
        region7_Cb = Cb.submat(new Rect(region7_pointA, region7_pointB));
        region8_Cb = Cb.submat(new Rect(region8_pointA, region8_pointB));
        region9_Cb = Cb.submat(new Rect(region9_pointA, region9_pointB));
    }

    @Override
    public Mat processFrame(Mat input){
        /*
         * Convert the input frame to HSV and threshold based on the Alliance color.
         */
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound, upperBound;
        if(Alliance.equals("Blue"))
        {
            lowerBound = new Scalar(100, 150, 50);
            upperBound = new Scalar(140, 255, 255);
        }
        else // assume Red
        {
            lowerBound = new Scalar(0, 150, 50);
            upperBound = new Scalar(10, 255, 255);
        }
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBound, upperBound, mask);

        /*
         * Compute the number of non-zero (matching) pixels in each submat region.
         */
        int count1 = Core.countNonZero(mask.submat(new Rect(region1_pointA, region1_pointB)));
        int count2 = Core.countNonZero(mask.submat(new Rect(region2_pointA, region2_pointB)));
        int count3 = Core.countNonZero(mask.submat(new Rect(region3_pointA, region3_pointB)));
        int count4 = Core.countNonZero(mask.submat(new Rect(region4_pointA, region4_pointB)));
        int count5 = Core.countNonZero(mask.submat(new Rect(region5_pointA, region5_pointB)));
        int count6 = Core.countNonZero(mask.submat(new Rect(region6_pointA, region6_pointB)));
        int count7 = Core.countNonZero(mask.submat(new Rect(region7_pointA, region7_pointB)));
        int count8 = Core.countNonZero(mask.submat(new Rect(region8_pointA, region8_pointB)));
        int count9 = Core.countNonZero(mask.submat(new Rect(region9_pointA, region9_pointB)));

        /*
         * Draw a rectangle showing each sample region on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);
        Imgproc.rectangle(input, region4_pointA, region4_pointB, BLUE, 2);
        Imgproc.rectangle(input, region5_pointA, region5_pointB, BLUE, 2);
        Imgproc.rectangle(input, region6_pointA, region6_pointB, BLUE, 2);
        Imgproc.rectangle(input, region7_pointA, region7_pointB, BLUE, 2);
        Imgproc.rectangle(input, region8_pointA, region8_pointB, BLUE, 2);
        Imgproc.rectangle(input, region9_pointA, region9_pointB, BLUE, 2);

        /*
         * Find the region with the maximum count of matching pixels.
         */
        int maxCount = count1;
        DeepPosition maxPosition = DeepPosition.LeftTop;
        if(count2 > maxCount){ maxCount = count2; maxPosition = DeepPosition.MiddleTop; }
        if(count3 > maxCount){ maxCount = count3; maxPosition = DeepPosition.RightTop; }
        if(count4 > maxCount){ maxCount = count4; maxPosition = DeepPosition.LeftMiddle; }
        if(count5 > maxCount){ maxCount = count5; maxPosition = DeepPosition.MiddleMiddle; }
        if(count6 > maxCount){ maxCount = count6; maxPosition = DeepPosition.RightMiddle; }
        if(count7 > maxCount){ maxCount = count7; maxPosition = DeepPosition.LeftBottom; }
        if(count8 > maxCount){ maxCount = count8; maxPosition = DeepPosition.MiddleBottom; }
        if(count9 > maxCount){ maxCount = count9; maxPosition = DeepPosition.RightBottom; }

        /*
         * Now that we found the region with the most matching pixels,
         * we actually need to go and figure out which sample region that
         * value was from
         */
        if(maxPosition == DeepPosition.LeftTop)
        {
            position = DeepPosition.LeftTop;
            Imgproc.rectangle(input, region1_pointA, region1_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.MiddleTop)
        {
            position = DeepPosition.MiddleTop;
            Imgproc.rectangle(input, region2_pointA, region2_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.RightTop)
        {
            position = DeepPosition.RightTop;
            Imgproc.rectangle(input, region3_pointA, region3_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.LeftMiddle)
        {
            position = DeepPosition.LeftMiddle;
            Imgproc.rectangle(input, region4_pointA, region4_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.MiddleMiddle)
        {
            position = DeepPosition.MiddleMiddle;
            Imgproc.rectangle(input, region5_pointA, region5_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.RightMiddle)
        {
            position = DeepPosition.RightMiddle;
            Imgproc.rectangle(input, region6_pointA, region6_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.LeftBottom)
        {
            position = DeepPosition.LeftBottom;
            Imgproc.rectangle(input, region7_pointA, region7_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.MiddleBottom)
        {
            position = DeepPosition.MiddleBottom;
            Imgproc.rectangle(input, region8_pointA, region8_pointB, YELLOW, -1);
        }
        else if(maxPosition == DeepPosition.RightBottom)
        {
            position = DeepPosition.RightBottom;
            Imgproc.rectangle(input, region9_pointA, region9_pointB, YELLOW, -1);
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }



    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
        public SkystonePosition getAnalysis()
        {
            return position;
        }
}