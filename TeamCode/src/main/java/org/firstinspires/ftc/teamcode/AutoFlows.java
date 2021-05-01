package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BananaPipeline;
import org.firstinspires.ftc.teamcode.util.CvCam;
import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AutoFlows {
	private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode

	final double tile = 0.6;
	int mul;
	Location startingPosition;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location firstPos; // -0.25,0.73
	Location shootPos;
	Location parkPos;
	Location wobbleFirst_pos;

	public AutoFlows(LinearOpMode opMode, Alliance alliance) {
		this.opMode = opMode;

		if (alliance == Alliance.BLUE) {
			mul = blue;
		} else {
			mul = red;
		}
		startingPosition = new Location(0.75 * mul, 0);
		a_pos = new Location(1.4 * mul, 1.45);
		b_pos = new Location(0.75 * mul, 2.15);
		c_pos = new Location(1.4 * mul, 2.65);
		firstPos = new Location(0.27 * mul, 0.73); // -0.25,0.73
		shootPos = new Location(0.20 * mul, 1.13);
		parkPos = new Location(0.8 * mul, 2);
		wobbleFirst_pos = new Location(2.5 * tile * mul, 2 * tile);

		robot = new DriveClass(this.opMode, DriveClass.ROBOT.COBALT, startingPosition).useEncoders();
		game = new GameClass(this.opMode);    // Declare OpMode members.

	}

	BananaPipeline pipeline;
	OpenCvCamera cam;

	public enum Alliance {BLUE, RED}

	public enum FlowType {WALL, BRIDGE, SHORT, PARK_ONLY}


	private DriveClass robot;
	private GameClass game;

	public HardwareMap hardwareMap = null;


	private ElapsedTime runtime = new ElapsedTime();

	final int blue = -1;
	final int red = 1;

	public Auto.ABC getRingNum(BananaPipeline pipeline) {
		if (pipeline.getTargetRect() == null) {
			return (Auto.ABC.A);
		} else {
			Rect rect = pipeline.getTargetRect();
			if (rect.height < rect.width / 2) {
				return (Auto.ABC.B);
			} else {
				return (Auto.ABC.C);
			}
		}
	}


	private void initCamera() {
		cam = CvCam.getCam(this.opMode.hardwareMap, true);
		pipeline = new BananaPipeline();
		cam.setPipeline(pipeline);
//        this.opMode.telemetry.addData("pipeline initialized", pipeline);
//        this.opMode.telemetry.update();
		cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
			}
		});
	}

	public void fullFlow() {
//        this.opMode.telemetry.addData("Status", "Initialized");
//        this.opMode.telemetry.update();

		initCamera();
		robot.init(this.opMode.hardwareMap);
		game.init(this.opMode.hardwareMap);


		game.initLifterPosition();
		game.setWobbleGrabber(false);
		game.initWobbleArmPosition();

		Auto.ABC abc = getRingNum(pipeline);
		this.opMode.telemetry.addData("Rings", abc);
		this.opMode.telemetry.update();

		// Wait for the game to start (driver presses PLAY)\
		this.opMode.waitForStart();
		runtime.reset();

		abc = getRingNum(pipeline);// a b c?
        this.opMode.telemetry.addData("Rings", abc);
        this.opMode.telemetry.update();

		game.wobbleArmGoTo(1500); //wobble up
		game.setSuperPosition(true);// fire position

		double heading = robot.getHeading();

		//go shoot
		robot.goToLocation(firstPos, 1, heading, 0.15);
		robot.goToLocation(shootPos, 1, 1.25, 0.01);
		game.update();
		// robot.turnTo(20, 0.6);

		while (opMode.opModeIsActive() && !game.getSuperState()) ;

		for (int x = 0; x < 3; x++) { // fire ring
			game.update();
			this.opMode.sleep(1200);
			game.update();
			game.shoot();
			game.update();
		}

		this.opMode.sleep(1000);
		game.setSuperPosition(false); //intake
		// robot.turnTo(0, 0.6);
        this.opMode.telemetry.addData("going to", abc);
        this.opMode.telemetry.update();


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


		//drop #1wobble
		//Last current position - tiles: (x: -0.5, y: 4.5)
		game.wobbleArmGoTo(5778);
		this.opMode.sleep(1000);
		game.setWobbleGrabber(true);
		this.opMode.sleep(350);

		// going to pick up the second wobble rod
		robot.goToLocation(wobbleFirst_pos, 1, heading, 0.1);
		robot.turnTo(164, 1);
		robot.drive(0.40, 0, 1, 180, false);

		// picking up the wobble
		game.wobbleArmGoTo(6500);
		opMode.sleep(300);
		game.setWobbleGrabber(false);
		opMode.sleep(700);
		game.wobbleArmGoTo(4000);

		// going back
		robot.drive(-0.40, 0, 1, 170, false);
		opMode.sleep(150);
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
		opMode.sleep(400);
		game.setWobbleGrabber(true);
		opMode.sleep(350);

		if (abc == Auto.ABC.A) {
			robot.drive(-0.05, 0.25, 1, heading, true, 0.1);
		}

		game.wobbleArmGoTo(100);

		robot.goToLocation(parkPos, 1, heading, 0.05);
		game.setWobbleGrabber(false);
	}

}
