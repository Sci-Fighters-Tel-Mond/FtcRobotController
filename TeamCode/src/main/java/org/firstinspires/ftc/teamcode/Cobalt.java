package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group = "Cobalt")
//@Disabled
public class Cobalt extends LinearOpMode {
	final double tile = 0.6;

	int lifterPosition_PowerShot;
	int lifterPosition_Goal;
	int lifterPosition_aShooter;

	double targetHeading_PowerShot;
	double targetHeading_Goal;
	double targetHeading_aShooter;

	double targetHeadingPowerShot2 = 4;
	double targetHeadingPowerShot3 = 8;

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	Location startingPosition = new Location(0 * tile, 0 * tile); //last x = -1.75*tile, y = 0*tile
	private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
	private GameClass game = new GameClass(this);

	private Toggle armShooter = new Toggle();
	private Toggle intake = new Toggle();

	private Toggle aShooter = new Toggle();

	private Toggle reverseIntake = new Toggle();
	private Toggle wobbleForward = new Toggle();
	private Toggle wobbleBackward = new Toggle();
	private Toggle wobbleGrabber = new Toggle(false);
	private Toggle shootHeading = new Toggle();
	private Toggle ringFire = new Toggle();
	private Toggle turningToggle = new Toggle();
	private Toggle wiperToggle = new Toggle(false);
	private Toggle lifterUp = new Toggle();
	private Toggle lifterDown = new Toggle();
	private Toggle powerShot = new Toggle();

	private double targetHeading = 0;

	enum Team {
		BLUE,
		RED
	}

	private Team team = Team.BLUE;

	void setPositionsAndHeadings(Team team) {
		this.team = team;

		if (team == Team.BLUE) {
			drive.resetOrientation(90);

			// X  BLUE
			lifterPosition_Goal = 1815;
			targetHeading_Goal = 2;

			// Y BLUE
			lifterPosition_PowerShot = 1700;
			targetHeading_PowerShot = 32;   // left power shoot

			// A BLUE
			lifterPosition_aShooter = 1840;
			targetHeading_aShooter = 31;

		} else if (team == Team.RED) {
			drive.resetOrientation(-90);

			// X RED
			lifterPosition_Goal = 1820;
			targetHeading_Goal = 4;

			// Y RED
			lifterPosition_PowerShot = 1700;
			targetHeading_PowerShot = -3;   // left power shoot


			// A RED
			lifterPosition_aShooter = 1820;
			targetHeading_aShooter = 35;
		}
	}

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		drive.init(hardwareMap);
		game.init(hardwareMap);

		game.initLifterPosition();
		game.initWobbleArmPosition();

		setPositionsAndHeadings(Team.BLUE);

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		drive.resetOrientation(90); //default blue

		runtime.reset();

		int turningCount = 0;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {

			boolean resetOrientation = gamepad1.start;

			if (resetOrientation) {
				if (gamepad1.x) {
					setPositionsAndHeadings(Team.BLUE);
				}
				if (gamepad1.y) {
					setPositionsAndHeadings(Team.RED);
				}
				drive.resetPosition();
				targetHeading = drive.getHeading();
				continue;
			}

//			boolean stopAll = gamepad1.a || gamepad2.a;

			armShooter.update(gamepad1.x || gamepad2.x); // up armShooter
			intake.update(gamepad1.dpad_right); // down armShooter, // gamepad2 isn't required

			aShooter.update(gamepad1.a || gamepad2.a);

			reverseIntake.update(gamepad1.dpad_left); // gamepad2 isn't required
			wobbleForward.update(gamepad1.dpad_up);
			wobbleBackward.update(gamepad1.dpad_down);
			wobbleGrabber.update(gamepad1.b);
			shootHeading.update(gamepad1.back || gamepad2.back);
			ringFire.update(gamepad1.right_bumper || gamepad2.right_bumper);
			wiperToggle.update(gamepad1.left_bumper || gamepad2.left_bumper);
			lifterDown.update(gamepad2.dpad_down);
			lifterUp.update(gamepad2.dpad_up);

			boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
			boolean dpadUp    = gamepad1.dpad_up || gamepad2.dpad_up;
			boolean dpadLeft  = gamepad1.dpad_left || gamepad2.dpad_left;
			boolean powerSet  = gamepad1.y || gamepad2.y;
			powerShot.update(powerSet && (dpadLeft || dpadUp || dpadRight)  ); //TODO: check powerShot toggle
//			boolean fieldOriented = !gamepad1.y;
			double boost = gamepad1.right_trigger * 0.6 + 0.4;

			double y    = -gamepad1.left_stick_y * boost;
			double x    = gamepad1.left_stick_x * boost;
			double turn = gamepad1.right_stick_x * boost;

			turningToggle.update(Math.abs(turn) > 0.02);

			if (turningToggle.isReleased()) {
				turningCount = 8;
			}
			if (!turningToggle.isPressed()) {
				turningCount--;
			}

			if (turningCount == 0) {
				targetHeading = drive.getHeading();
			}

			if (!turningToggle.isPressed() && turningCount < 0) {
				double delta = drive.getDeltaHeading(targetHeading);
				if (Math.abs(delta) > 0.1) {
					double gain = 0.018;
					turn = delta * gain;
				}
			}

//			drive.setPowerOriented(y, x, turn, fieldOriented);
			drive.setPowerOriented(y, x, turn, true);

			if (powerSet) {
				if (powerShot.isClicked()) { // y
					game.setLifterTargetPosition(lifterPosition_PowerShot);

					double heading = targetHeading_PowerShot;
					if (dpadUp) {
						heading += targetHeadingPowerShot2;
					} else if (dpadRight) {
						heading += targetHeadingPowerShot3;
					}
					targetHeading = heading;

					game.setSuperPosition(true);
				}
				continue;
			}

			if (shootHeading.isClicked()) {
				drive.turnTo(3, 1);
			}

			if (ringFire.isClicked()) {
				game.setRingMover(0);
				sleep(300);
				game.setRingMover(1);
			}

			if (wobbleBackward.isClicked()) {
				game.wobbleArmGoTo(2850);
				// game.setWobbleArm(-0.6);
			} //else if (wobbleBackward.isReleased()){
			//game.setWobbleArm(0.0);
			//game.wobbleArmGoTo(3000);

			// }

			if (wobbleForward.isClicked()) {
				game.wobbleArmGoTo(6185);
				//game.setWobbleArm(0.6);
			}//} else if (wobbleForward.isReleased()) {
			//game.setWobbleArm(0);
			//}


			if (wobbleGrabber.isChanged()) {
				game.setWobbleGrabber(wobbleGrabber.getState());
			}


			if (intake.isClicked()) {
				game.setSuperPosition(false);
			}

			if (reverseIntake.isPressed()) {
				game.setIntakePower(-1);
			} else {
				if (reverseIntake.isReleased()) {
					game.setIntakePower(0);
				}
			}

			if (wiperToggle.isPressed()) {
				game.setWipers(wiperToggle.getState());
			}
			telemetry.addData("Wiper changed", wiperToggle.getState());

//			if (stopAll) {
//				game.stopAll();
//			}

			if(lifterUp.isClicked()) {
				game.setLifterTargetPosition(game.getLifterTargetPosition() + 10);
			}

			if(lifterDown.isClicked()) {
				game.setLifterTargetPosition(game.getLifterTargetPosition() - 10);
			}

			if (aShooter.isClicked()) { // a
				game.setLifterTargetPosition(lifterPosition_aShooter);
				targetHeading = targetHeading_aShooter;
				game.setSuperPosition(true);
			}

			if (armShooter.isClicked()) { // x
				game.setLifterTargetPosition(lifterPosition_Goal);
				//targetHeading = targetHeading_Goal;
				game.setSuperPosition(true);
			}

			game.lifterMoveManually(-gamepad2.right_stick_y/4);

			telemetry.addData("X Pos", drive.getPosX());
			telemetry.addData("Y Pos", drive.getPosY());
			telemetry.addData("Heading", drive.getHeading());
			telemetry.addData("Target", targetHeading);
			telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));

			game.update();
			telemetry.update();
		}
	}
}
