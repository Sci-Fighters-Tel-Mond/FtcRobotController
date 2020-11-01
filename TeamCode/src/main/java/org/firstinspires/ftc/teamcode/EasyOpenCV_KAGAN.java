package org.firstinspires.ftc.teamcode;

import android.widget.ArrayAdapter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTimeoutException;
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

@TeleOp
public class EasyOpenCV_KAGAN extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BananaPipeline pipeline;

    static private class BananaPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        double ratio = -1;

        public double get_ratio() {return ratio;}

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
            Scalar blue_min = new Scalar(170, 100, 100);
            Scalar blue_max = new Scalar(198, 100, 100);
            Scalar red_min = new Scalar(0, 255, 255);
            Scalar red_max = new Scalar(12, 255, 255);
            Scalar green_min = new Scalar(80, 255, 255);
            Scalar green_max = new Scalar(103, 223, 255);

            Core.inRange(hsv, yellow_min, yellow_max, mask);
            frame.setTo(new Scalar(255,123,132), mask);

            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat her = new Mat();
            Imgproc.findContours(mask, contours, her, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            //Core.bitwise_not(mask, mask);
            //frame.setTo(new Scalar(123, 32, 45), mask);

            for (int num = 0; num < contours.size(); num++) {

                Mat cnt =  contours.get(num);
                double area = Imgproc.contourArea(cnt);

                if (1000 < area){
                    Imgproc.drawContours(frame, contours, num, new Scalar(2, 100, 150), 3);
                    Rect rect = Imgproc.boundingRect(cnt);

                    ratio = rect.height / rect.width;
                    Imgproc.rectangle(frame,rect,new Scalar(95,100,150),1);
                    break;
                }
            }
            return frame;
        }

    }


    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCV_KAGAN.BananaPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
}