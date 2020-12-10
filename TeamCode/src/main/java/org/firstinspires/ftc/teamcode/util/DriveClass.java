// Drive Class 2
// Ultimate Goal 2020 - 2021

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriveClass {
	//region DON'T TOUCH
	private boolean useEncoders = false;
	private boolean useBrake = false;

	public DriveClass useEncoders() {
		this.useEncoders = true;
		return this;
	}

	public DriveClass useBrake() {
		this.useBrake = true;
		return this;
	}
	//endregion DON'T TOUCH

	private LinearOpMode opMode; // TODO: maybe volatile?, Firs I declared it as OpMode now its LinearOpMode

	volatile private DcMotorEx fl_Drive = null;
	volatile private DcMotorEx fr_Drive = null;
	volatile private DcMotorEx bl_Drive = null;
	volatile private DcMotorEx br_Drive = null;

	private BNO055IMU imu = null; // TODO: maybe volatile?

	private int fl_startPos = 0;
	private int fr_startPos = 0;
	private int bl_startPos = 0;
	private int br_startPos = 0;

	public DriveClass(LinearOpMode opMode) {
		this.opMode = opMode;
	}

	public void init(HardwareMap hw) {
		//region get from hw
		fl_Drive = hw.get(DcMotorEx.class, "fl_drive");
		fr_Drive = hw.get(DcMotorEx.class, "fr_drive");
		bl_Drive = hw.get(DcMotorEx.class, "bl_drive");
		br_Drive = hw.get(DcMotorEx.class, "br_drive");
		//endregion get from hw

		//region setDirection
		fl_Drive.setDirection(DcMotorEx.Direction.REVERSE);
		fr_Drive.setDirection(DcMotorEx.Direction.FORWARD);
		bl_Drive.setDirection(DcMotorEx.Direction.REVERSE);
		br_Drive.setDirection(DcMotorEx.Direction.FORWARD);
		//endregion setDirection

		//region setMode
		if (useEncoders) {
			fl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			fr_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			bl_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			br_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		} else {
			fl_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			fr_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			bl_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			br_Drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		}
		//endregion setMode

		//region setZeroPowerBehavior
		if (useBrake) {
			fl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			fr_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			bl_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			br_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}
		//endregion setZeroPowerBehavior
	}

	private void initIMU() {
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

		imu.initialize(parameters);

		opMode.telemetry.addData("Gyro", "calibrating...");
		opMode.telemetry.update();

		ElapsedTime timer = new ElapsedTime();
		timer.reset();
		while (!imu.isGyroCalibrated() && !opMode.isStopRequested() && timer.seconds() < 12) {
			opMode.sleep(1100);
		}
		if (imu.isGyroCalibrated()) {
			opMode.telemetry.addData("Gyro", "Done Calibrating");
		} else {
			opMode.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
		}

		imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

		opMode.telemetry.update();

		RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
		RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
	}

	public double getImuDistance(Position target) {
		Position current = imu.getPosition();
		double dx = current.x - target.x;
		double dy = current.y - target.y;
		double sqrt = Math.pow(dy, 2) + Math.pow(dx, 2);
		return Math.sqrt(sqrt);
	}

	public void setPower(double forward, double turn, double strafe) {
		fl_Drive.setPower(forward + turn + strafe);
		fr_Drive.setPower(forward - turn - strafe);
		bl_Drive.setPower(forward + turn - strafe);
		br_Drive.setPower(forward - turn + strafe);
	}

	public void stopPower() {
		setPower(0, 0, 0);
	}

	public double getHeading() {
		Orientation orientation = imu.getAngularOrientation();
		return -orientation.firstAngle;
	}

	public double getDeltaHeading(double target) {
		double currentAngle = getHeading();
		double delta = target - currentAngle;

		if (delta < 180) {
			delta = delta + 360;
		}
		if (delta > 180) {
			delta = delta - 360;
		}

		return delta;
	}

	public void resetPosition() {
		fl_startPos = fl_Drive.getCurrentPosition();
		fr_startPos = fr_Drive.getCurrentPosition();
		bl_startPos = bl_Drive.getCurrentPosition();
		br_startPos = br_Drive.getCurrentPosition();
	}

	public double getForwardDistance() {
		final double polsMeter = 1150;
		int fl_tick = fl_Drive.getCurrentPosition() - fl_startPos;
		int fr_tick = fr_Drive.getCurrentPosition() - fr_startPos;
		int bl_tick = bl_Drive.getCurrentPosition() - bl_startPos;
		int br_tick = br_Drive.getCurrentPosition() - br_startPos;
		double fl_dist = fl_tick / polsMeter;
		double fr_dist = fr_tick / polsMeter;
		double bl_dist = bl_tick / polsMeter;
		double br_dist = br_tick / polsMeter;
		return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
	}


	public void driveForward(double meter, double power) {
		double targetAngle = getHeading(); // zeroAngle
		driveForward(meter, power, targetAngle);
	}

	public void driveForward(double meter, double targetPower, double targetAngle) {
		double s = (meter < 0) ? -1 : 1;
		resetPosition();

		while ((getForwardDistance() * s < meter * s) && opMode.opModeIsActive()) {
			double power = targetPower;
			double acclGain = 2;
			double acclPower = Math.abs(getForwardDistance()) * acclGain + 0.2;
			if (acclPower < power)
				power = acclPower;

			double breakgain = 1;
			double deltaForward = Math.abs(meter - getForwardDistance());
			double breakpower = deltaForward * breakgain;
			if (breakpower < power)
				power = breakpower;

			double currentAngle = getHeading();
			double err = getDeltaHeading(targetAngle);
			double gain = 0.040;
			double correction = gain * err;

			setPower(power * s, correction, 0);

			opMode.telemetry.addData("deltaForward", deltaForward);
			opMode.telemetry.addData("acclPower", acclPower);
			opMode.telemetry.addData("breakPower", breakpower);
			opMode.telemetry.addData("power", power);

			opMode.telemetry.addData("target", targetAngle);
			opMode.telemetry.addData("current", currentAngle);
			opMode.telemetry.addData("Error", err);
			opMode.telemetry.update();
		}
		stopPower();
	}

	public double getStrafeDistance() {
		double polsMeter = 1000;

		int fl_tick = fl_Drive.getCurrentPosition() - fl_startPos;
		int fr_tick = fr_Drive.getCurrentPosition() - fr_startPos;
		int bl_tick = bl_Drive.getCurrentPosition() - bl_startPos;
		int br_tick = br_Drive.getCurrentPosition() - br_startPos;

		double leftFrontDist = fl_tick / polsMeter;
		double rightFrontDist = fr_tick / polsMeter;
		double leftBackDist = bl_tick / polsMeter;
		double rightBackDist = br_tick / polsMeter;
		// double forwardDistance = (leftBackDist + rightBackDist + rightFrontDist + leftFrontDist) / 4;
		return (leftBackDist + rightBackDist - rightFrontDist + leftFrontDist) / 4;
	}

	public void strafe(double meter, double power) {
		double targetAngle = getHeading(); // zeroAngle
		strafe(meter, power, targetAngle);
	}

	public void strafe(double meter, double power, double targetAngle) {
		double s = (meter < 0) ? -1 : 1;

		resetPosition();

		while ((getStrafeDistance() * s < meter * s) && opMode.opModeIsActive()) {
			double currentAngle = getHeading();
			double err = getDeltaHeading(targetAngle);
			double gain = 0.035;
			double correction = gain * err;

			setPower(0, correction, power * s);

			opMode.telemetry.addData("distance", getStrafeDistance());
			opMode.telemetry.addData("target", targetAngle);
			opMode.telemetry.addData("current", currentAngle);
			opMode.telemetry.addData("Error", err);
			opMode.telemetry.update();
		}
		stopPower();
	}

	public void turn(double deg, double power) {
		double targetAngle = getHeading() + deg; // zeroAngle
		turnTo(targetAngle, power);
	}

	public void turnTo(double targetAngle, double targetPower) {
		double delta = getDeltaHeading(targetAngle);
		double s = (delta < 0) ? -1 : 1;
		while ((delta * s > 5 * s) && opMode.opModeIsActive()) {

			delta = getDeltaHeading(targetAngle);
			double gain = 0.04;
			double power = gain * delta * targetPower;
			if (Math.abs(power) < 0.1)
				power = 0.1 * Math.signum(power);

			setPower(0, power, 0);

			opMode.telemetry.addData("target", targetAngle);
			opMode.telemetry.addData("current", getHeading());
			opMode.telemetry.addData("delta", delta);
			opMode.telemetry.addData("power", power);
			opMode.telemetry.update();

		}
		stopPower();
	}
}
