package org.firstinspires.ftc.teamcode.study;

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
public class OpenCV_Barak extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BananaPipeline pipeline;

    static private class BananaPipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat yellowMask = new Mat();
        Mat greenMask = new Mat();
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
            //color scalar - get three double values (Hue, Saturation, Value)
            Scalar yellow_min = new Scalar(23, 100, 130); //color scalar
            Scalar yellow_max = new Scalar(55, 255, 255); //color scalar

            Core.inRange(hsv, yellow_min, yellow_max, yellowMask); //range detection & conversion (yellow) ---

            //frame.setTo(new Scalar(0,0,0), mask); //setting colors...

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
        pipeline = new OpenCV_Barak.BananaPipeline();
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