/* Copyright (c) 2017 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(group = "Linear Opmode")
//@Disabled
public class DrivingTest extends LinearOpMode {
//    BananaPipeline pipeline;
//    OpenCvInternalCamera phoneCam;
    private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.SCORPION, new Location(0, 0)).useEncoders();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Toggle fieldOriented = new Toggle(false);
    final double tile = 0.6;
    final int left = -1;
    final int right = 1;

    private void initCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new BananaPipeline();
//        phoneCam.setPipeline(pipeline);

//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
                //phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
    }

    // main functions

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


//        initCamera();
        drive.init(hardwareMap);
        drive.resetPosition();

        waitForStart();

        while(opModeIsActive()) {

            double leftPower;
            double rightPower;

            double boost =  gamepad1.right_trigger * 0.4 + 0.6;
            double y     = -gamepad1.left_stick_y * boost;
            double x     =  gamepad1.left_stick_x * boost;
            double turn  =  gamepad1.right_stick_x * boost;

            boolean fieldOriented = gamepad1.right_bumper != true;

//            double targetX = 0;
//            if(gamepad1.a) { // ring tracking
//                if(pipeline.getTargetRect() != null) {
//                    Rect rect = pipeline.getTargetRect();
//                    int xCenter = rect.x + rect.width/2;
//                    int screenCenter = pipeline.width/2;
//                    int delta = xCenter - screenCenter;
//                    targetX = delta;
//                    double gain = 0.7;
//                    double k = 2.0 / pipeline.width;  // transform from pixels to power (-1...1).
//                    double correction = k * delta * gain;
//                    turn += correction;
//                }
//            }

            drive.setPowerOriented(y, x, turn, false);


            if (gamepad1.x){
                drive.resetOrientation(0);

            }

            if (gamepad1.y){
                drive.resetPosition();
            }

            if (gamepad1.b){
                drive.goTo(-2 * tile, 5 * tile, 0.8, drive.getHeading(), 0.05);
            }


            telemetry.addData("X: ", drive.getAbsolutesPosX());
            telemetry.addData("Y:", drive.getAbsolutesPosY());


            telemetry.addData("Dx: ", drive.getStrafeDistance());
            telemetry.addData("Dy:", drive.getForwardDistance());

//            telemetry.addData("target", targetX);
//            telemetry.addData("turn", turn);
//            telemetry.addData("width", pipeline.width);
            telemetry.update();
        }
    }
}
