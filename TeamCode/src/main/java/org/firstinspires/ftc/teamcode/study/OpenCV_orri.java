package org.firstinspires.ftc.teamcode.study;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
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
import java.util.List;


@TeleOp(group = "Study")
@Disabled
public class OpenCV_orri extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BananaPipline pipeline;
    volatile private Point point = new Point();

    private class BananaPipline extends OpenCvPipeline {


        Mat hsv = new Mat();
        Mat mask = new Mat();

        @Override
        public void init(Mat firstFrame) {
            super.init(firstFrame);
        }

        @Override
        public Mat processFrame(Mat frame) {

            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, new Scalar(25, 100, 100), new Scalar(45, 255, 255), mask);

            frame.setTo(new Scalar(0, 0, 0), mask);

            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat moti = new Mat();
            Imgproc.findContours(frame, contours, moti, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double biggestArea = 0;
            Rect rectang = new Rect();

            for (int i = 0; i < contours.size(); i++) {
                Mat cnt = contours.get(i);
                double area = Imgproc.contourArea(cnt);

                if (area > 1000) {
                    if (biggestArea < area) {
                        biggestArea = area;
                        Imgproc.drawContours(frame, contours, i, new Scalar(0, 0, 255), 2);
                        rectang = Imgproc.boundingRect(contours.get(i));
                    }
                }

                Imgproc.rectangle(frame, rectang, new Scalar(255, 0, 0));

            }
 
            return frame;
        }
    }

    public void getTarget() {
        Point point = new Point();
    }

    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BananaPipline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
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
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}