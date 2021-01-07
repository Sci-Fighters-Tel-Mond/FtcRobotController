/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(group = "Linear Opmode")
//@Disabled
public class DrivingTest extends LinearOpMode {
    BananaPipeline pipeline;
    OpenCvInternalCamera phoneCam;
    private DriveClass drive = new DriveClass(this).useEncoders();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Toggle fieldOriented = new Toggle(false);
    final double tile = 0.6;
    final int left = -1;
    final int right = 1;

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BananaPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    // 0 - a
    // 1 -b
    // 4 - c
    enum ABC {A, B, C};
    public ABC getRingNum(BananaPipeline pipeline) {
        if (pipeline.getTargetRect() == null) {
            return (ABC.A);
        } else {
            Rect rect = pipeline.getTargetRect();
            if (rect.height < rect.width / 2) {
                return (ABC.B);
            } else {
                return (ABC.C);
            }
        }
    }
    //limits the max and min of a certain value
    private double minmax(double max, double min, double value) {
        if(value > max) {
            value = max;
        }
        if(value < min) {
            value = min;
        }
        return value;
    }

    // main functions

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        initCamera();
        drive.init(hardwareMap);
        drive.resetPosition();

        waitForStart();

        while(opModeIsActive()) {

            double leftPower;
            double rightPower;

            double boost = gamepad1.right_trigger * 0.4 + 0.6;
            double forward = -gamepad1.left_stick_y * boost;
            double turn = gamepad1.right_stick_x * boost;
            double strafe = gamepad1.left_stick_x * boost;
            boolean fieldOriented = !gamepad1.right_bumper;

            double targetX = 0;

            if(gamepad1.a){
                if(pipeline.getTargetRect() != null) {
                    Rect rect = pipeline.getTargetRect();
                    int x = rect.x + rect.width/2;
                    int screenCenter = pipeline.width/2;
                    int delta = x - screenCenter;
                    targetX = delta;
                    double gain = 0.7;
                    double k = 2.0 / pipeline.width;  // transform from pixels to power (-1...1).
                    double correction = k * delta * gain;
                    turn += correction;
                }
            }


            if ( fieldOriented != true) {
                drive.setPower(forward, turn, strafe);
            } else {
                double alpha = -drive.getHeading() / 180 * Math.PI;
                double strait = forward * Math.cos(alpha) - strafe * Math.sin(alpha);
                double side = forward * Math.sin(alpha) + strafe * Math.cos(alpha);
                drive.setPower(strait, turn, side);
            }

            if (gamepad1.x){
                drive.resetOrientation();

            }

            if (gamepad1.y){
                drive.resetPosition();
            }

            if (gamepad1.b){
                drive.goTo(-2 * tile, 5 * tile, 0.8, drive.getHeading());
            }


            telemetry.addData("X: ", drive.getPosX());
            telemetry.addData("Y:", drive.getPosY());


            telemetry.addData("Dx: ", drive.getStrafeDistance());
            telemetry.addData("Dy:", drive.getForwardDistance());

//            telemetry.addData("target", targetX);
//            telemetry.addData("turn", turn);
//            telemetry.addData("width", pipeline.width);
            telemetry.update();
        }
    }
}
