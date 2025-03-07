package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public abstract class IMRec extends LinearOpMode {

    public class ImgPipe extends OpenCvPipeline {

        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();
        Mat maskYellow = new Mat();
        Mat mask = new Mat();
        Mat edge = new Mat();
        Mat max = new Mat();
        Mat hier = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
//        List<MatOfPoint2f> rectangles = new ArrayList<>();
        List<MatOfPoint> Non2fRectangles = new ArrayList<>();
        Scalar Rlower = new Scalar(0,0,140);
        Scalar Rupper = new Scalar(30,50,255);
        Scalar Blower = new Scalar(140,46,0);
        Scalar Bupper = new Scalar(210,110,5);
        Scalar Ylower = new Scalar(0,140,180);
        Scalar Yupper = new Scalar(5,230,255);

        double epsilon;
        MatOfPoint2f tempMat2f;
        MatOfPoint2f approx;


        @Override
        public Mat processFrame(Mat input) {
            Non2fRectangles.clear();
            contours.clear();
            max.empty();
//            input.copyTo(org);


            Core.inRange(input,Rlower,Rupper,maskRed);
            Core.inRange(input,Blower,Bupper,maskBlue);
            Core.inRange(input,Ylower,Yupper,maskYellow);


            Core.add(maskRed,maskBlue,maskYellow,mask);
            Imgproc.GaussianBlur(mask,mask, new Size(11,11),0);

            Core.bitwise_and(input,input,mask,mask);
            Imgproc.cvtColor(mask,mask,Imgproc.COLOR_BGR2GRAY);

            Imgproc.findContours(edge, contours,hier,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);

//            input.setTo(new Scalar(255,255,255),mask);
//            Core.bitwise_not(mask,mask);

//            input.setTo(new Scalar(0,0,0),mask);
//            Imgproc.GaussianBlur(input,input, new Size(101,101),0);
//            Imgproc.threshold(input,input,100,255,Imgproc.THRESH_BINARY);
//            Imgproc.Canny(input,edge,30,70);



//            Imgproc.drawContours(org,contours,-1,new Scalar(0,255,0),3);
            if(!contours.isEmpty()){
//                max = contours.get(0);
                for (int i = 0; i < contours.size() && !opModeIsActive() && !isStopRequested(); i++){
                    if(Imgproc.contourArea(contours.get(i))>1000){
                        tempMat2f = new MatOfPoint2f(contours.get(i).toArray());
                        epsilon = 0.05 * Imgproc.arcLength(tempMat2f,true);
                        Imgproc.approxPolyDP(tempMat2f,approx,epsilon,true);
                        if(approx.toArray().length==4){
//                            rectangles.add(tempMat2f);
                            Non2fRectangles.add(new MatOfPoint(tempMat2f.toArray()));
                        }
                    }
                }
            }


            Imgproc.drawContours(input,Non2fRectangles,-1,new Scalar(255,0,0),3);
//            if(!max.empty()){
//                size = Imgproc.boundingRect(max);
//                midx = size.x+size.width/2;
//                if(!opModeIsActive()){
//                    if(midx<input.size().width/3){
//                        telemetry.addLine("Left detected");
//                    }else if (midx>input.size().width*2/3){
//                        telemetry.addLine("Right detected    ");
//                    }else{
//                        telemetry.addLine("Middle detected");
//                    }
//                }
//            }

//            if(max_list.size()!=0){
//                Imgproc.drawContours(org,max_list,-1,new Scalar(0,255,0),3);
//            }
//            telemetry.addData("X pos",midx);
            telemetry.update();
//            org.release();
            maskBlue.release();
            maskRed.release();
            maskYellow.release();
            mask.release();
            edge.release();
            max.release();
            hier.release();
            tempMat2f.release();
            approx.release();
//            org.empty();
            mask.empty();
            maskBlue.empty();
            maskRed.empty();
            maskYellow.empty();
            edge.empty();
            max.empty();
            hier.empty();
            tempMat2f.empty();
            approx.empty();
            return input;
        }
    }
}
