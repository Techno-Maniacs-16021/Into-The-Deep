package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Detects the yellow “sample” (Into-The-Deep season) and returns
 *   • forward distance  (inches from the lens)
 *   • lateral offset    (inches, +right  /  –left)
 */
public class SampleDistancePipeline extends OpenCvPipeline {

    /* ───--- CALIBRATION CONSTANTS (fill in once) ---───────────────────────── */
    private static final double REAL_WIDTH_IN  = 3.80;   // sample’s real width
    private static final double FOCAL_PX       = 640.0;  // ← compute in Part A

    /* ───--- COLOUR THRESHOLDS (HSV) ---────────────────────────────────────── */
    // Yellow band – tweak if your field lighting is odd
    private final Scalar Y_LOWER = new Scalar( 20, 100, 100);
    private final Scalar Y_UPPER = new Scalar( 35, 255, 255);

    /* ───--- WORK BUFFERS ---───────────────────────────────────────────────── */
    private final Mat hsv    = new Mat();
    private final Mat mask   = new Mat();
    private final Mat blur   = new Mat();
    private final Mat hier   = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();

    /* ───--- OUTPUTS ---────────────────────────────────────────────────────── */
    private volatile double forwardIn  = Double.NaN;   // Z (towards targets)
    private volatile double lateralIn  = Double.NaN;   // X (left – / right +)

    /** Your robot code calls these from another thread (e.g. in OpMode loop) */
    public double getForwardInch() { return forwardIn; }
    public double getLateralInch() { return lateralIn; }

    /* ───--- OpenCvPipeline required method ---────────────────────────────── */
    @Override
    public Mat processFrame(Mat input) {

        // 1. Threshold yellow in HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, Y_LOWER, Y_UPPER, mask);

        // 2. Clean up mask
        Imgproc.GaussianBlur(mask, blur, new Size(11,11), 0);

        // 3. Find contours
        contours.clear();
        Imgproc.findContours(blur, contours, hier,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // 4. Pick the largest contour (most likely the sample)
        double maxArea = 0;
        Rect bestRect  = null;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(c);
            }
        }

        if (bestRect != null) {
            // 5a. Draw a box so you see what we’re tracking
            Imgproc.rectangle(input,
                    new Point(bestRect.x, bestRect.y),
                    new Point(bestRect.x+bestRect.width, bestRect.y+bestRect.height),
                    new Scalar(0,255,0), 3);

            // 5b. Calculate forward distance (Z) and lateral offset (X)
            double pxWidth = bestRect.width;

            // Forward (in) from similar-triangles
            forwardIn = (REAL_WIDTH_IN * FOCAL_PX) / pxWidth;

            // Lateral (in): shift of centre in pixels → inches at Z
            double cxPx   = bestRect.x + bestRect.width  / 2.0;
            double frameCx = input.cols() / 2.0;
            double dxPx   = cxPx - frameCx;
            lateralIn = (dxPx * forwardIn) / FOCAL_PX;

        } else {
            // Nothing found → mark NaN so robot code knows data is invalid
            forwardIn = Double.NaN;
            lateralIn = Double.NaN;
        }

        return input;  // The Driver Station preview shows the green box
    }

    @Override
    public void close() {
        hsv.release(); mask.release(); blur.release(); hier.release();
    }
}
