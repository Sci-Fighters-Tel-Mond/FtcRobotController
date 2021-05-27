package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GameClass {

	private LinearOpMode opMode;

	private DcMotorEx shooter = null;
	public DcMotorEx lifter = null;
	private DcMotorEx intake = null;

	private DcMotorEx wobbleArm = null;
	private Servo wobbleGrabber1 = null;
	private Servo wobbleGrabber2 = null;
	private DigitalChannel wobbleArmLimiter = null;
	private DigitalChannel lifterLimiter = null;

	private Servo ringMover = null; // 1 - inside, 0 - outside

	private Toggle superState = new Toggle();// true - shooterPosition
	private Toggle shooterState = new Toggle();
	private Toggle intakeState = new Toggle();
	private Toggle wobbleGrabberState = new Toggle();
	private Toggle testLifterToggle = new Toggle();

	private enum LifterRequest {UP, DOWN, STAY};
	private LifterRequest lifterRequest = LifterRequest.STAY;

	final private double shooterSpeed = 0.9;
	final private int lifterUpTargetPosition = 1840;
	final private int lifterDownTargetPosition = 0;


	private ElapsedTime timer = new ElapsedTime();


	public GameClass(LinearOpMode opMode) {
		this.opMode = opMode;
	}

	public void init(HardwareMap hw) {
		//region get from hw
		shooter = hw.get(DcMotorEx.class, "shooter");
		lifter = hw.get(DcMotorEx.class, "lifter");
		intake = hw.get(DcMotorEx.class, "collector");

		wobbleArm = hw.get(DcMotorEx.class, "wobble");
		wobbleGrabber1 = hw.get(Servo.class, "wobble_grabber1");
		wobbleGrabber2 = hw.get(Servo.class, "wobble_grabber2");
		wobbleArmLimiter = hw.get(DigitalChannel.class, "wobble_limiter");
		lifterLimiter = hw.get(DigitalChannel.class, "shooter_limiter");

		ringMover = hw.get(Servo.class, "ring_mover");
		//endregion get from hw

		//region setDirection
		intake.setDirection(DcMotorEx.Direction.REVERSE);
		lifter.setDirection(DcMotorEx.Direction.REVERSE);

		wobbleArm.setDirection(DcMotorEx.Direction.REVERSE);
		//endregion setDirection

		//region encoders
		shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		//endregion encoders

		lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		ringMover.setPosition(1);
	}

	public void setSuperPosition(boolean goUp) {
		if (goUp) {
			setIntake(false);
			setShooterRoller(true);
			lifterUpDown(true); // up
			if (getWobbleArmPos() <= 400) {
				wobbleArmGoTo(3000);
			}

		} else { // goDown
			setShooterRoller(false); // stop shooter
			lifterUpDown(false); // down
		}
	}

	public boolean getSuperState(){
		update();
		return superState.getState();
	}

	public void lifterUpDown(boolean goUp) {
		if (goUp) {
			lifter.setTargetPosition(lifterUpTargetPosition);
			lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			lifter.setPower(1);

			lifterRequest = LifterRequest.UP;
		} else {
			lifter.setTargetPosition(lifterDownTargetPosition);
			lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			lifter.setPower(1);

			lifterRequest = LifterRequest.DOWN;

		}
		timer.reset();
	}

	public void lifterMove(int goUp) {
		int lifterCurruntPosition = lifter.getCurrentPosition();
		if (goUp > 0) {
			lifter.setTargetPosition(lifterCurruntPosition + goUp);
			lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			lifter.setPower(1);

			//lifterRequest = LifterRequest.UP;
			opMode.telemetry.addData("lifter going up", goUp);
		} else {
			lifter.setTargetPosition(lifterCurruntPosition + goUp);
			lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			lifter.setPower(1);

			//lifterRequest = LifterRequest.DOWN;
			opMode.telemetry.addData("lifter going down", goUp);
		}
		timer.reset();
	}

	public void update() {
		opMode.telemetry.addData("wobble position", getWobbleArmPos());

		opMode.telemetry.addData("Lifter pos", lifter.getCurrentPosition());
		opMode.telemetry.addData("shooter!!!!!!!!!!!", shooter.getVelocity());

		if (lifterRequest == LifterRequest.UP){
			if (lifter.getCurrentPosition() > lifterUpTargetPosition  || timer.milliseconds() > 4000 ){
				lifterRequest = LifterRequest.STAY;
				superState.set(true);
			}
			opMode.telemetry.addData("Lifter GO UP", timer.milliseconds());
		}

		if (lifterRequest == LifterRequest.DOWN) {
			if (getLifterLimiter() || (timer.milliseconds() > 4000)) {
				lifterRequest = LifterRequest.STAY;
				setIntake(true);
				lifter.setPower(0);
				superState.set(false);
			}
			opMode.telemetry.addData("Lifter GO Down", timer.milliseconds());
		}

		opMode.telemetry.addData("Super State", superState.getState());
	}

	public void lifterTest(double pow) {
		testLifterToggle.update(Math.abs(pow) > 0.2);
		if (testLifterToggle.isClicked()) {
			lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		if (testLifterToggle.isPressed()) {
			int curTicks = lifter.getCurrentPosition();
			if ((curTicks > 2800 && pow > 0) || (getLifterLimiter() && pow < 0)) {
				pow = 0;
			}
			lifter.setPower(pow);
			opMode.telemetry.addData("TEST Lifter Power", pow);
		} else if (testLifterToggle.isReleased()) {
			opMode.telemetry.addData("TEST Lifter Power", pow);
			lifter.setPower(0);
		}
	}

	public void initWobbleArmPosition(){
		wobbleArm.setPower(-0.6);
		ElapsedTime time = new ElapsedTime();
		while (getWobbleArmLimiter() == false){
			if ( time.milliseconds() > 5000) break;
		}
		wobbleArm.setPower(0);
		opMode.sleep(500);
		wobbleArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	public int getWobbleArmPos(){
		return wobbleArm.getCurrentPosition();
	}

	public void wobbleArmGoTo(int position) {
		wobbleArm.setTargetPosition(position);
		wobbleArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		wobbleArm.setPower(1);
	}

	public void initLifterPosition() {
		if (getLifterLimiter() == false) {
			lifter.setPower(-1);
			ElapsedTime timer = new ElapsedTime();
			while (getLifterLimiter() == false) {
				if (timer.milliseconds() > 4000) break;
			}
			lifter.setPower(0);
			opMode.sleep(1000);

		}
		lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		superState.set(false);
	}


	public boolean getLifterLimiter() {
		return !lifterLimiter.getState();
	}


	private void setShooterRoller(boolean active) {
		shooterState.set(active);
		shooter.setPower(active ? shooterSpeed : 0);
	}

	private void toggleShooter() {
		boolean state = shooterState.toggle();
		setShooterRoller(state);
	}


	private void setIntake(boolean active) {
		intakeState.set(active);
		intake.setPower(active ? 1 : 0);
	}

	public void setWobbleArm(double pow) {
		if (wobbleArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
			wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		if (getWobbleArmLimiter()) {
			pow = Math.max(pow, 0);
		}
		if (getWobbleArmPos() >= 10000) {
			pow = Math.min(pow, 0);
		}
		wobbleArm.setPower(pow);
	}

	public void setWobbleGrabber(boolean state) {
		wobbleGrabberState.set(state);
		wobbleGrabber1.setPosition(state ? 1 : 0);
		wobbleGrabber2.setPosition(state ? 1 : 0);
	}

	public void toggleWobbleGrabber() {
		boolean state = wobbleGrabberState.toggle();
		setWobbleGrabber(state);
	}

	public boolean getWobbleArmLimiter() {
		return !wobbleArmLimiter.getState();
	}

	public void setRingMover(double amt) {
		if (superState.getState()) {
			ringMover.setPosition(amt);
		}
	}

	public void shoot() {
		if (superState.getState() ) {
			setRingMover(0);
			opMode.sleep(300);
			setRingMover(1);
		}
	}

	public void setIntakePower(double v) {
		intake.setPower(v);
	}

	public void stopAll() {
		superState.set(false);
		setShooterRoller(false);
		setIntake(false);

		wobbleArm.setPower(0);
		lifter.setPower(0);
	}
}
