// Drive Class
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

	private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode

	volatile private DcMotorEx fl = null;
	volatile private DcMotorEx fr = null;
	volatile private DcMotorEx bl = null;
	volatile private DcMotorEx br = null;

	private BNO055IMU imu = null;

	private int fl_startPos = 0;
	private int fr_startPos = 0;
	private int bl_startPos = 0;
	private int br_startPos = 0;

	public DriveClass(LinearOpMode opMode) {
		this.opMode = opMode;
	}

	public void init(HardwareMap hw) {
		//region get from hw
		fl = hw.get(DcMotorEx.class, "fl");
		fr = hw.get(DcMotorEx.class, "fr");
		bl = hw.get(DcMotorEx.class, "bl");
		br = hw.get(DcMotorEx.class, "br");
		//endregion get from hw

		//region setDirection
		fl.setDirection(DcMotorEx.Direction.REVERSE);
		fr.setDirection(DcMotorEx.Direction.FORWARD);
		bl.setDirection(DcMotorEx.Direction.REVERSE);
		br.setDirection(DcMotorEx.Direction.FORWARD);
		//endregion setDirection

		fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		//region setMode
		if (useEncoders) {
			fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		} else {
			fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
			br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		}
		//endregion setMode

		//region setZeroPowerBehavior
		if (useBrake) {
			fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}
		//endregion setZeroPowerBehavior

		initIMU(hw);
	}

	private void initIMU(HardwareMap hw) {
		imu = hw.get(BNO055IMU.class, "imu");

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

		imu.initialize(parameters);

		opMode.telemetry.addData("Gyro", "calibrating...");
		opMode.telemetry.update();

		ElapsedTime timer = new ElapsedTime();
		timer.reset();
		while (!imu.isGyroCalibrated() && !opMode.isStopRequested() && timer.seconds() < 5) {
			opMode.sleep(50);
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
		fl.setPower(forward + turn + strafe);
		fr.setPower(forward - turn - strafe);
		bl.setPower(forward + turn - strafe);
		br.setPower(forward - turn + strafe);
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

	public double getForwardDistance() {
		final double polsMeter = 2455;
		int fl_tick = fl.getCurrentPosition() - fl_startPos;
		int fr_tick = fr.getCurrentPosition() - fr_startPos;
		int bl_tick = bl.getCurrentPosition() - bl_startPos;
		int br_tick = br.getCurrentPosition() - br_startPos;
		double fl_dist = fl_tick / polsMeter;
		double fr_dist = fr_tick / polsMeter;
		double bl_dist = bl_tick / polsMeter;
		double br_dist = br_tick / polsMeter;
		return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
	}

	public double getPosX(){
		double polsMeter = 2587;

		int fl_tick = fl.getCurrentPosition();
		int fr_tick = fr.getCurrentPosition();
		int bl_tick = bl.getCurrentPosition();
		int br_tick = br.getCurrentPosition();

		double leftFrontDist = fl_tick / polsMeter;
		double rightFrontDist = fr_tick / polsMeter;
		double leftBackDist = bl_tick / polsMeter;
		double rightBackDist = br_tick / polsMeter;
		return (-leftBackDist + rightBackDist - rightFrontDist + leftFrontDist) / 4;
	}

	public double getPosY(){
		final double polsMeter = 2455;
		int fl_tick = fl.getCurrentPosition();
		int fr_tick = fr.getCurrentPosition();
		int bl_tick = bl.getCurrentPosition();
		int br_tick = br.getCurrentPosition();
		double fl_dist = fl_tick / polsMeter;
		double fr_dist = fr_tick / polsMeter;
		double bl_dist = bl_tick / polsMeter;
		double br_dist = br_tick / polsMeter;
		return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
	}

	public double getStrafeDistance() {
		double polsMeter = 2587;

		int fl_tick = fl.getCurrentPosition() - fl_startPos;
		int fr_tick = fr.getCurrentPosition() - fr_startPos;
		int bl_tick = bl.getCurrentPosition() - bl_startPos;
		int br_tick = br.getCurrentPosition() - br_startPos;

		double leftFrontDist = fl_tick / polsMeter;
		double rightFrontDist = fr_tick / polsMeter;
		double leftBackDist = bl_tick / polsMeter;
		double rightBackDist = br_tick / polsMeter;
		return (-leftBackDist + rightBackDist - rightFrontDist + leftFrontDist) / 4;
	}

	public void resetPosition() {
		fl_startPos = fl.getCurrentPosition();
		fr_startPos = fr.getCurrentPosition();
		bl_startPos = bl.getCurrentPosition();
		br_startPos = br.getCurrentPosition();
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

	public void goTo(double x, double y, double targetPower, double targetHeading){
		double currentX = getPosX();
		double currentY = getPosY();
		double deltaX = x - currentX;
		double deltaY = y - currentY;
		drive(deltaY, deltaX, targetPower, targetHeading);
	}

	public void drive(double forward, double sideward, double targetPower, double targetAngle) {
		double sf = (forward < 0) ? -1 : 1;
		double ss = (sideward < 0) ? -1 : 1;
		double c = Math.sqrt(sideward * sideward + forward * forward);
		double RVf = forward / c;
		double RVs = sideward / c;
		final double minPower = 0.2;

		resetPosition();

		while (opMode.opModeIsActive() && (RVf != 0) ||  (RVs != 0)) {

			if (getForwardDistance() * sf > forward * sf) {
				RVf = 0;
			}
			if (getStrafeDistance() * ss > sideward * ss) {
				RVs = 0;
			}
			double power = targetPower ;

			double deltaForward = forward - getForwardDistance();
			double deltaStrafe  = sideward - getStrafeDistance();

			double deltaC = Math.sqrt(deltaStrafe * deltaStrafe + deltaForward * deltaForward);
			double lengthC = c - deltaC ;

			double acclGain = 2;
			double acclPower = lengthC * acclGain + minPower;

			if (acclPower < power) {
				power = acclPower;
			}

			double breakgain = 2;
			double breakPower = deltaC * breakgain + minPower;

			if (breakPower < power)  {
				power = breakPower;
			}


			double err = getDeltaHeading(targetAngle);
			double gain = 0.040;
			double correction = gain * err;
			double Vf = RVf * power;
			double Vs = RVs * power;

			setPower(Vf, correction, Vs);

//			opMode.telemetry.addData("delta forward:", deltaForward);
//			opMode.telemetry.addData("speed forward:", Vf);
//			opMode.telemetry.addData("delta strafe:", deltaStrafe);
//			opMode.telemetry.addData("speed strafe:", Vs);
//			opMode.telemetry.addData("power:", power);

			//position Telemetry:
			opMode.telemetry.addData("x position:", getPosX());
			opMode.telemetry.addData("y position:", getPosY());

			opMode.telemetry.update();
		}
		stopPower();
	}

}
