package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BananaPipeline extends OpenCvPipeline {

    Mat hsv;
    Mat mask;
    Mat subMat;
    Rect subRect;
    public int width;
    public int height;

    volatile private Point targetPos = null;
    //Rect(x, y, width, height)             x, y - position. width, height - dimensions.
    volatile private Rect targetRect = null;

    public Point getTargetPos() {
        return targetPos;
    }

    public Rect getTargetRect() {
        return targetRect;
    }

    @Override
    public void init(Mat firstFrame) {
        super.init(firstFrame);


        mask = new Mat();
        hsv = new Mat();
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);

        width = hsv.width();
        height = hsv.height();
        subRect = new Rect(new Point(width*2/5,height*5/12), new Point(width*3/5, height*6/12));
        subMat = hsv.submat(subRect);
    }

    @Override
    public Mat processFrame(Mat frame) {
        // search for the orange Rings
//        Mat smallFrame = new Mat(frame, new Range(frame.height() / 2, frame.height()));
        Mat smallFrame = frame;
        Imgproc.cvtColor(smallFrame, hsv, Imgproc.COLOR_RGB2HSV);  // Convert to HSV color set
        Scalar min_yellow = new Scalar(8, 130, 160);
        Scalar max_yellow = new Scalar(60, 255, 255);
        Core.inRange(hsv, min_yellow, max_yellow, mask );   // Mask all orange


        Mat kernel = Mat.ones(5,5, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel, new Point(2,2)  );

        smallFrame.setTo(new Scalar(0, 0, 0), mask);


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw all of the contours.
        // for (int i = 0; i < contours.size(); i++) {
        //    Imgproc.drawContours(frame, contours, i, new Scalar(0, 200, 200), 1);
        // }

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
            Imgproc.rectangle(smallFrame, pt1, pt2, new Scalar(255, 0, 0), 2);

            int x = rects.get(biggestIndex).x + width / 2;
            int y = rects.get(biggestIndex).y + height / 2;
            targetPos = new Point(x, y);
            targetRect = rects.get(biggestIndex);
        } else {
            targetPos = null;
            targetRect = null;
        }

        // show the sub Rect for color testing
        Scalar c = Core.mean(subMat);  // Calcs the average color in the sub Rect area
        Imgproc.rectangle(smallFrame, subRect, new Scalar(0, 255, 0), 2); // Draw a green rect around the subRect area
        Point textPoint = new Point(10, subRect.y - 20);
        Imgproc.putText(smallFrame, String.format("H: %03.1f, S: %03.1f, V: %03.1f", c.val[0], c.val[1], c.val[2]), textPoint, Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0,255,00), 2);

        return smallFrame;
    }

}
