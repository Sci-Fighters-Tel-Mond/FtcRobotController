package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;

@TeleOp
public class OpenCv_Start extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BananaPipeline pipeline;



    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BananaPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void runOpMode() {

        setupCamera();

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }





    public static class BananaPipeline extends OpenCvPipeline {

        private Random rng = new Random(12345);
        Mat hsv;
        Mat black;
        Mat mask;

        Mat hsvRegion;

        static final Point A = new Point(100,100);
        static final Point B = new Point(150, 150);

        @Override
        public void init(Mat firstFrame) {
            super.init(firstFrame);

            // black = new Mat.zeros(firstFrame.size(), firstFrame.type());
            black = new Mat(firstFrame.rows(), firstFrame.cols(), firstFrame.type());
            mask = new Mat();
            hsv = new Mat();
            Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);
            hsvRegion = hsv.submat(new Rect(A, B));
        }

        @Override
        public Mat processFrame(Mat frame) {
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
            Scalar min_yellow = new Scalar(23, 100, 130);
            Scalar max_yellow = new Scalar(45, 255, 255);
            Core.inRange(hsv, min_yellow, max_yellow, mask);

            Core.bitwise_and(frame, black, frame, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy,  Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < contours.size(); i++) {
                // Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                // Imgproc.drawContours(frame, contours, i, color, 1);
                Imgproc.drawContours(frame, contours, i, new Scalar(0,200,200), 1);
            }


            Imgproc.rectangle(frame, A, B, new Scalar(0,0,255), 1);
            Scalar av = Core.mean(hsvRegion);
            Imgproc.putText(frame, String.format("HSV: %3.0f, %3.0f, %3.0f", av.val[0], av.val[1], av.val[2]), new Point(20,20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 200, 0));



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

            int biggestIndex = -1;
            double biggestArea = 0;

            for (int i = 0; i < rects.size(); i++) {
                if (rects.get(i).area() > biggestArea) {
                    biggestIndex = i;
                    biggestArea = rects.get(i).area();
                }
            }

            int width = rects.get(biggestIndex).width;
            int height = rects.get(biggestIndex).height;


            return frame;
        }
    }
}