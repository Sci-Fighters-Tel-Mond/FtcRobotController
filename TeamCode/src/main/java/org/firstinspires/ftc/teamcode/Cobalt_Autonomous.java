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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Full Flaw - Uno Robot", group = "Cobalt")
//@Disabled
public class Cobalt_Autonomous extends LinearOpMode {
    BananaPipeline pipeline;
    OpenCvInternalCamera phoneCam;
    private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.COBALT, new Location(Location.LOCATION.BLUE_EXTERNAL_START_POSITION, -1.75*0.6, 0)).useEncoders().useBrake();
    private GameClass game = new GameClass(this);
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Toggle wobbleGrabber = new Toggle();
    private Toggle reverseIntake = new Toggle();

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
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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


    // main functions
    // main functions
    // main functions
    // main functions
    // main functions

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        initCamera();
        robot.init(hardwareMap);

        ABC abc = getRingNum(pipeline);
        telemetry.addData("Rings", abc);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        abc = getRingNum(pipeline);
        telemetry.addData("Rings", abc);
        telemetry.update();

        double heading = robot.getHeading();

        robot.drive(2.5 * tile, 0, 1, heading, false);


//        if (abc == ABC.A) {
////            robot.diagonal(tile * 2.5, tile / 3 * left, 0.8, heading);
////            robot.drive(5 * tile, 3 * tile * left - 0.2, 0.8, heading);
//            robot.goTo(-2 * tile, 3 * tile, 0.8, heading);
//
////            while (gamepad1.b == false){;}
////            robot.goTo(-1*tile, 4 * tile, 0.8, heading);
////
////            while (gamepad1.b == false){;}
////            robot.goTo(-2* tile, 5 * tile, 0.8, heading);
////
////            while (gamepad1.b == false){;}
////            robot.goTo(0,0, 0.8, heading);
//
//        }
//
//        if (abc == ABC.B) {
////            robot.driveForward(tile * 4, 1, heading);
////            robot.strafe(tile, 1);
//            robot.goTo(-1*tile, 4 * tile, 0.8, heading);
//
//        }
//
//        if (abc == ABC.C) {
//            robot.goTo(-2* tile, 5 * tile, 0.8, heading);
////            robot.driveForward(tile * 5, 1, heading);
//        }

//
        while(opModeIsActive()) {

            double leftPower;
            double rightPower;
            double boost = gamepad1.right_trigger * 0.4 + 0.6;
            double drive = -gamepad1.left_stick_y * boost;
            double turn = gamepad1.right_stick_x * boost;
            double strafe = gamepad1.left_stick_x * boost;

            if (gamepad1.right_bumper == true) {
                robot.setPower(drive, turn, strafe);
            } else {
                double alpha = -robot.getHeading() / 180 * Math.PI;
                double forward = drive * Math.cos(alpha) - strafe * Math.sin(alpha);
                double side = drive * Math.sin(alpha) + strafe * Math.cos(alpha);
                robot.setPower(forward, turn, side);
            }
            telemetry.addData("X", robot.getForwardDistance());
            telemetry.addData("Y", robot.getStrafeDistance());

            telemetry.addData("x position:", robot.getAbsolutesPcsX());
            telemetry.addData("y position:", robot.getAbsolutesPosY());

            telemetry.update();
        }
    }
}
