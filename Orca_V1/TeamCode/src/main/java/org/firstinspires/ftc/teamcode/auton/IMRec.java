package org.firstinspires.ftc.teamcode.auton;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
    public class IMRec extends OpenCvPipeline {

        Rect size = new Rect();
        Mat org = new Mat();
        Mat maskYellow = new Mat();
        double midx;
        double midy;

        Mat edge = new Mat();
        Rect max = new Rect();
        Mat hier = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

//        List<MatOfPoint> Non2fRectangles = new ArrayList<>();
        Scalar Rlower = new Scalar(0,0,140);
        Scalar Rupper = new Scalar(30,50,255);
        Scalar Blower = new Scalar(140,46,0);
        Scalar Bupper = new Scalar(210,110,5);
        Scalar Ylower = new Scalar(25/2.0,55,30);
        Scalar Yupper = new Scalar(60/2.0,255,255);

        Scalar black = new Scalar(255,255,255);
        Scalar white = new Scalar(0,0,0);
        Size blur = new Size(21,21);
        Point bottomLeft = new Point(0,0);
        Point topRight = new Point(0,0);
        @Override
        public Mat processFrame(Mat input) {


            input.copyTo(org);

            Imgproc.cvtColor(org,org,Imgproc.COLOR_RGB2HSV);
            Core.inRange(org,Ylower,Yupper,maskYellow);


            org.setTo(black,maskYellow);
            Core.bitwise_not(maskYellow,maskYellow);
            org.setTo(white,maskYellow);
            Imgproc.GaussianBlur(org,org, blur,0);

            Imgproc.threshold(org,org,150,255,Imgproc.THRESH_BINARY);


            Imgproc.Canny(org,edge,30,70);
            Imgproc.findContours(edge, contours,hier,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);

            if(!contours.isEmpty()){
                max = Imgproc.boundingRect(contours.get(0));

                for (int i = 0; i < contours.size()-1; i++){
                    if(Imgproc.boundingRect(contours.get(i)).area() > max.area()){
                        max = Imgproc.boundingRect(contours.get(i));
                    }
                }
            }
            Imgproc.drawContours(input,contours,-1,black,3);


            if(!max.empty()) {
                size = max;
                midx = (size.x + size.width / 2.0)/input.size().width;
                midy = (size.y + size.height / 2.0)/input.size().height;
//                telemetry.addLine("BEST PT: "+midx+","+midy);
                bottomLeft.x = size.x;
                bottomLeft.y = size.y;
                topRight.x = size.x + size.width;
                topRight.y = size.y + size.height;

                Imgproc.rectangle(input, bottomLeft, topRight, new Scalar(255,0,0), 5);
            }
//            Core.add(maskBlue,mask,mask);



//            Core.bitwise_and(input,input,mask,mask);
//            Imgproc.cvtColor(mask,mask,Imgproc.COLOR_BGR2GRAY);

//            Imgproc.findContours(mask, contours,hier,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);

//            input.setTo(new Scalar(255,255,255),mask);
//            Core.bitwise_not(mask,mask);

//            input.setTo(new Scalar(0,0,0),mask);
//            Imgproc.GaussianBlur(input,input, new Size(101,101),0);
//            Imgproc.threshold(input,input,100,255,Imgproc.THRESH_BINARY);
//            Imgproc.Canny(input,edge,30,70);



//            Imgproc.drawContours(org,contours,-1,new Scalar(0,255,0),3);
//            if(!contours.isEmpty()){
////                max = contours.get(0);
//                for (int i = 0; i < contours.size(); i++){
//                    if(Imgproc.contourArea(contours.get(i))>1000){
//                        tempMat2f = new MatOfPoint2f(contours.get(i).toArray());
//                        epsilon = 0.05 * Imgproc.arcLength(tempMat2f,true);
//                        Imgproc.approxPolyDP(tempMat2f,approx,epsilon,true);
//                        if(approx.toArray().length==4){
////                            rectangles.add(tempMat2f);
//                            Non2fRectangles.add(new MatOfPoint(tempMat2f.toArray()));
//                        }
//                    }
//                }
//            }


//            Imgproc.drawContours(input,Non2fRectangles,-1,new Scalar(255,0,0),3);
//            Imgproc.cvtColor(input,input,Imgproc.COLOR_HSV2RGB);
//            org.release();
//            org.empty();


            edge.release();
//            edge.empty();
//            max.release();
//            max.empty();
            hier.release();
            org.release();
            maskYellow.release();

//            hier.empty();
            contours.clear();
//            telemetry.update();
            return input;

//            maskBlue.release();
//            maskRed.release();
//            maskYellow.release();
//            mask.release();
//            edge.release();
//            max.release();
//            hier.release();
//            tempMat2f.release();
//            approx.release();
//
////            org.empty();
//            mask.empty();
//            maskBlue.empty();
//            maskRed.empty();
//            maskYellow.empty();
//            edge.empty();
//            max.empty();
//            hier.empty();
////            tempMat2f.empty();
//            approx.empty();
//            return mask;
        }

        public double getMidX(){
            return midx;
        }
        public double getMidY(){
            return midy;

    }}

