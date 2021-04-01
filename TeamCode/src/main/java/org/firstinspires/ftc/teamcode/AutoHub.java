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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Linear Opmode")
//@Disabled
public class AutoHub extends LinearOpMode {
	final double tile = 0.6;
	BananaPipeline pipeline;
	OpenCvWebcam cam;

	Location startingPosition = new Location(Location.LOCATION.BLUE_EXTERNAL_START_POSITION,-1.75*tile,0*tile);
	Location a_pos = new Location(Location.LOCATION.BLUE_A,-1.2,1.3);
	Location b_pos = new Location(Location.LOCATION.BLUE_B,-0.8,1.9);
	Location c_pos = new Location(Location.LOCATION.BLUE_C,-1.2,2.5);
	Location shootPos = new Location(Location.LOCATION.BLUE_SHOOTING_POINT,-0.96,1.255);

//    private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders();
//    private GameClass  game  = new GameClass(this);    // Declare OpMode members.

	private ElapsedTime runtime = new ElapsedTime();

	final int left = -1;
	final int right = 1;

	private void initCamera() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");
		cam = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
		pipeline = new BananaPipeline();
		cam.setPipeline(pipeline);

		cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
			cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
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


	// main functions ==============================================================================

	@Override
	public void runOpMode() {

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		initCamera();
//        robot.init(hardwareMap);
//        game.init(hardwareMap);

//        ABC abc = getRingNum(pipeline);
//        telemetry.addData("Rings", abc);
//        telemetry.update();

//        game.initLifterPosition();
//        game.initWobbleArmPosition();

		// Wait for the game to start (driver presses PLAY)
		ABC abc = getRingNum(pipeline);
		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {

			abc = getRingNum(pipeline);// a b c?
//            telemetry.addData("Rings", abc);
//            telemetry.update();

//        game.wobbleArmGoTo(1500); //wobble up
//        sleep(200);
//        game.setSuperPosition(true);// fire position

//        double heading = robot.getHeading();

//        robot.goTo(shootPos.x, shootPos.y, 1, 0);
//        game.update();
//        robot.turnTo(20, 0.6);

//        while (! game.getSuperState()){

//        }

//        for (int x = 0; x < 3; x++) { // fire ring
//           game.shoot();
//           sleep(1000);
//        }

//        robot.turnTo(0, 0.6);
//            telemetry.addData("going to", abc);
//            telemetry.update();

			if (abc == ABC.A) {
//            robot.goTo(a_pos.x,a_pos.y, 0.8,heading);
				telemetry.addData("Ring", "A");
			}

			if (abc == ABC.B) {
//            robot.goTo(b_pos.x,b_pos.y, 0.8,heading);
				telemetry.addData("Ring", "B");
			}

			if (abc == ABC.C) {
//            robot.goTo(c_pos.x,c_pos.y, 0.8,heading);
				telemetry.addData("Ring", "C");
			}

			telemetry.update();

			//Last current position - tiles: (x: -0.5, y: 4.5)
//        game.wobbleArmGoTo(5778);
//        sleep(3000);
//        game.setWobbleGrabber(true);
//        sleep(250);
//        game.wobbleArmGoTo(100);
//        sleep(2000);
//        robot.goTo(-1,1.8,0.8,heading);
//        game.setWobbleGrabber(false);
		}
	}
}
