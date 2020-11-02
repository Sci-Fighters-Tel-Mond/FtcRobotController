package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BananaPipeline extends OpenCvPipeline {

    Mat hsv;
    Mat mask;

//        Mat hsvRegion;

//        static final Point A = new Point(100,100);
//        static final Point B = new Point(150, 150);

    @Override
    public void init(Mat firstFrame) {
        super.init(firstFrame);

        mask = new Mat();
        hsv = new Mat();
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);
    }

    @Override
    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar min_yellow = new Scalar(23, 100, 130);
        Scalar max_yellow = new Scalar(45, 255, 255);
        Core.inRange(hsv, min_yellow, max_yellow, mask);

        frame.setTo(new Scalar(0, 0, 0), mask);


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(frame, contours, i, new Scalar(0, 200, 200), 1);
        }


        ArrayList<Rect> rects = new ArrayList<Rect>();
        Rect rect = null;
        double maxArea = 300;
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea > maxArea) {
                rect = Imgproc.boundingRect(contours.get(i));
                rects.add(rect);
            }
        }

        int biggestIndex = 0;
        double biggestArea = 0;

        for (int i = 0; i < rects.size(); i++) {
            if (rects.get(i).area() > biggestArea) {
                biggestIndex = i;
                biggestArea = rects.get(i).area();
            }
        }

        if (rects.size() > 0) {
            int width = rects.get(biggestIndex).width;
            int height = rects.get(biggestIndex).height;
            Point pt1 = new Point(rects.get(biggestIndex).x, rects.get(biggestIndex).y);
            Point pt2 = new Point(rects.get(biggestIndex).x + width, rects.get(biggestIndex).y + height);
            Imgproc.rectangle(frame, pt1, pt2, new Scalar(255, 0, 0), 2);
        }

        return frame;
    }
}
