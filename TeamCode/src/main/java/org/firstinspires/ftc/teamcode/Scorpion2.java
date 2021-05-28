package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group = "Scorpion")
public class Scorpion2 extends LinearOpMode {
	final double tile = 0.6;

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	Location startingPosition = new Location(0 * tile, 0 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.SCORPION, startingPosition).useEncoders().useBrake();

	private Toggle turningToggle = new Toggle();

	private Toggle a_btn = new Toggle();
	private Toggle b_btn = new Toggle();
	private Toggle x_btn = new Toggle();
	private Toggle y_btn = new Toggle();

	private double targetHeading = 0;

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.init(hardwareMap);


		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		drive.resetOrientation(90); //default blue

		runtime.reset();

		int turningCount = 0;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			a_btn.update(gamepad1.a);
			b_btn.update(gamepad1.b);
			x_btn.update(gamepad1.x);
			y_btn.update(gamepad1.y);

			boolean resetOrientation = gamepad1.start;

			if (resetOrientation) {
				if (gamepad1.x) {
					drive.resetOrientation(90);
				}
				if (gamepad1.y) {
					drive.resetOrientation(-90);
				}
				drive.resetPosition();
				targetHeading = drive.getHeading();
				continue;
			}

			boolean fieldOriented = !gamepad1.left_bumper;
			double boost = gamepad1.right_trigger * 0.6 + 0.4;

			double y = -gamepad1.left_stick_y * boost;
			double x = gamepad1.left_stick_x * boost;
			double turn = gamepad1.right_stick_x * boost;

			turningToggle.update(Math.abs(turn) > 0.05);

			if (turningToggle.isReleased()) {
				turningCount = 5;
			}
			if (!turningToggle.isPressed()) {
				turningCount--;
			}

			if (turningCount == 0) {
				targetHeading = drive.getHeading();

			}

			if (!turningToggle.isPressed() && turningCount < 0) {
				double delta = drive.getDeltaHeading(targetHeading);
				if (Math.abs(delta) > 1) {
					double gain = 0.05;
					turn = delta * gain;
				}
			}

			if (a_btn.isClicked()) {
				drive.goTo(0., 0., 0.5, 0, 0.01);
			}
			if (b_btn.isClicked()) {
				drive.goTo(0.6, 0.6, 0.5, 60, 0.01);
			}
			if (x_btn.isClicked()) {
				drive.goTo(0.6, 0.0, 0.5, 0, 0.01);
			}
			if (y_btn.isClicked()) {
				drive.goTo(0.0, 0.6, 0.5, 60, 0.01);
			}

			drive.setPowerOriented(y, x, turn, fieldOriented);


			telemetry.addData("Abs Pos","X,Y %2.3f, %2.3f", drive.getAbsolutesPosX(), drive.getAbsolutesPosY());
			telemetry.addData("Pos", "x,y: %2.3f, %2.3f", drive.getPosX(), drive.getPosY());
			telemetry.addData("Heading", drive.getHeading());
			telemetry.addData("Target", targetHeading);
			telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));
			telemetry.update();
		}
	}
}
