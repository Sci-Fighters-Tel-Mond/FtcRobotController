package org.firstinspires.ftc.teamcode.study;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp
@Disabled
public class OpenCv_Ohad extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BananaPipeline pipeline;

    static private class BananaPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat yellowMask = new Mat();
        Mat greenMask = new Mat();
        Mat mask = new Mat();
        double ratio = -1;
        public volatile int x;
        public volatile int y;

        public double get_ratio() {
            return ratio;
        }

        @Override
        public void init(Mat firstFrame) {
            super.init(firstFrame);
        }


        @Override
        public Mat processFrame(Mat frame) {

            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
            //  H    S    V
            Scalar yellow_min = new Scalar(23, 100, 130);
            Scalar yellow_max = new Scalar(55, 255, 255);



            Core.inRange(hsv, yellow_min, yellow_max, yellowMask);


            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat her = new Mat();
            Imgproc.findContours(mask, contours, her, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Core.bitwise_not(mask, mask);
            frame.setTo(new Scalar(0, 0, 0), mask);

            int maxAreaIndex = -1;
            double maxArea = 0;

            Rect rect = null;
            for (int i = 0; i < contours.size(); i++) {

                Mat cnt = contours.get(i);
                double area = Imgproc.contourArea(cnt);

                Imgproc.drawContours(frame, contours, i, new Scalar(197, 123, 170), 1);

                if (1000 < area) {
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                        rect = Imgproc.boundingRect(cnt);
                        ratio = rect.height / rect.width;
                        x = rect.x + rect.width / 2;
                        y = rect.y + rect.height / 2;

                    }
                }
            }
            if (rect != null) {
                Imgproc.rectangle(frame, rect, new Scalar(95, 100, 150), 1);
            }

            return frame;
        }

    }


    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OpenCv_Ohad.BananaPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }


    @Override
    public void runOpMode() {

        setupCamera();

        waitForStart();

        while (opModeIsActive()) {

            //telemetry.addData("Analysis", pipeline.getAnalysis());
            //telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}