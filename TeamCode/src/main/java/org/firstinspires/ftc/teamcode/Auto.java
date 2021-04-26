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
import org.firstinspires.ftc.teamcode.util.CvCam;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Linear Opmode")
//@Disabled
public class Auto extends LinearOpMode {
	final double tile = 0.6;
	BananaPipeline pipeline;
	OpenCvCamera cam;
	Location startingPosition = new Location(-0.75, 0);
	Location a_pos = new Location(-1.35, 1.4);
	Location b_pos = new Location(-0.7, 1.95);
	Location c_pos = new Location(-1.35, 2.6);
	Location firstPos = new Location(-0.27, 0.73); // -0.25,0.73
	Location shootPos = new Location(-0.27, 1.4);
	Location parkPos = new Location(-0.8, 2);
	Location wobbleFirst_pos = new Location(-2.5 * tile, 2 * tile);

	private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders();
	private GameClass game = new GameClass(this);    // Declare OpMode members.

	private ElapsedTime runtime = new ElapsedTime();


	final int left = -1;
	final int right = 1;

	private void initCamera() {
		cam = CvCam.getCam(hardwareMap, true);
		pipeline = new BananaPipeline();
		cam.setPipeline(pipeline);

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
	enum ABC {A, B, C}

	;

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
		robot.init(hardwareMap);
		game.init(hardwareMap);

		ABC abc = getRingNum(pipeline);
		telemetry.addData("Rings", abc);
		telemetry.update();

		game.initLifterPosition();
		game.setWobbleGrabber(false);
		game.initWobbleArmPosition();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		abc = getRingNum(pipeline);// a b c?
		telemetry.addData("Rings", abc);
		telemetry.update();

		game.wobbleArmGoTo(1500); //wobble up
		game.setSuperPosition(true);// fire position

		double heading = robot.getHeading();

		robot.goToLocation(firstPos, 1, heading, 0.1);
		robot.goToLocation(shootPos, 1.1, heading, 0.01);
		game.update();
		// robot.turnTo(20, 0.6);

		while (opModeIsActive() && !game.getSuperState()) ;

		for (int x = 0; x < 3; x++) { // fire ring
			game.update();
			sleep(1000);
			game.update();
			game.shoot();
			game.update();
		}
		sleep(1000);
		game.setSuperPosition(false);
		// robot.turnTo(0, 0.6);
		telemetry.addData("going to", abc);
		telemetry.update();


		switch (abc) {
			case A:
				robot.goToLocation(a_pos, 1, heading, 0.05);
				break;
			case B:
				robot.goToLocation(b_pos, 1, heading, 0.05);
				break;
			case C:
				robot.goToLocation(c_pos, 1, heading, 0.05);
				break;
		}

		//Last current position - tiles: (x: -0.5, y: 4.5)
		game.wobbleArmGoTo(5778);
		sleep(1000);
		game.setWobbleGrabber(true);
		sleep(350);

		robot.goToLocation(wobbleFirst_pos, 1, heading, 0.1);
		robot.turnTo(164, 1);
		robot.drive(0.45, 0, 1, 180, false);

		game.wobbleArmGoTo(5600);
		sleep(200);
		game.setWobbleGrabber(false);
		sleep(700);
		game.wobbleArmGoTo(4000);

		robot.drive(-0.44, 0, 1, 170, false);
		sleep(150);
		robot.turnTo(0, 1);

		switch (abc) {
			case A:
				robot.goToLocation(a_pos, 1, heading, 0.05);
				break;
			case B:
				robot.goToLocation(b_pos, 1, heading, 0.05);
				break;
			case C:
				robot.goToLocation(c_pos, 1, heading, 0.05);
				break;
		}

		game.wobbleArmGoTo(5778);
		sleep(400);
		game.setWobbleGrabber(true);
		sleep(350);

		if (abc == ABC.A) {
			robot.drive(-0.05, 0.25, 1, heading, true, 0.1);
		}

		game.wobbleArmGoTo(100);

		robot.goToLocation(parkPos, 1, heading, 0.05);
		game.setWobbleGrabber(false);
	}
}
