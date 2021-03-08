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

public class DriveClass {
	//region DON'T TOUCH
	final double tile = 0.6;
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
	private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

	private boolean fieldOriented = true;
	private double angleOffset = 0;

	private int fl_startPos = 0;
	private int fr_startPos = 0;
	private int bl_startPos = 0;
	private int br_startPos = 0;

	public enum ROBOT {
		SCORPION,
		COBALT
	};

	private ROBOT robot;

	private Location startingPosition;

	private double forwardTicksPerMeter;
	private double strafeTicksPerMeter;

	ElapsedTime timer = new ElapsedTime();

	public DriveClass(LinearOpMode opMode, ROBOT robot, Location startingPosition) {
		this.opMode = opMode;
		this.robot = robot;
		this.startingPosition = startingPosition;


		if (robot == ROBOT.SCORPION) {
			this.forwardTicksPerMeter = 2455;
			this.strafeTicksPerMeter = 2587;
		} else if (robot == ROBOT.COBALT) {
			this.forwardTicksPerMeter = 1753;
			this.strafeTicksPerMeter = 2006;
		}
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

//		imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

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

	public void setPowerOriented(double y, double x, double turn, boolean fieldOriented){
		if ( fieldOriented != true) {
			setPower(y , turn, x);  // No field oriented
		} else {
			double phiRad = (-getHeading() + angleOffset) / 180 * Math.PI;
			double forward = y * Math.cos(phiRad) - x * Math.sin(phiRad);
			double strafe = y * Math.sin(phiRad) + x * Math.cos(phiRad);
			setPower(forward, turn, strafe);
		}

		opMode.telemetry.addData("front left:", fl.getCurrentPosition());
		opMode.telemetry.addData("front right:", fr.getCurrentPosition());
		opMode.telemetry.addData("back left:", bl.getCurrentPosition());
		opMode.telemetry.addData("back right:", br.getCurrentPosition());

	}

	public void stopPower() {
		setPower(0, 0, 0);
	}

	public void resetOrientation(double angle) {
		imu.initialize(parameters);
		angleOffset = angle;
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
		double fl_tick = fl.getCurrentPosition() - fl_startPos;
		double fr_tick = fr.getCurrentPosition() - fr_startPos;
		double bl_tick = bl.getCurrentPosition() - bl_startPos;
		double br_tick = br.getCurrentPosition() - br_startPos;
		double fl_dist = fl_tick / forwardTicksPerMeter;
		double fr_dist = fr_tick / forwardTicksPerMeter;
		double bl_dist = bl_tick / forwardTicksPerMeter;
		double br_dist = br_tick / forwardTicksPerMeter;
		return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
	}

	public double getPosY() {
		return getAbsolutesPosY() + startingPosition.y;
	}

	public double getAbsolutesPosY() {
		double fl_tick = fl.getCurrentPosition();
		double fr_tick = fr.getCurrentPosition();
		double bl_tick = bl.getCurrentPosition();
		double br_tick = br.getCurrentPosition();
		double fl_dist = fl_tick / forwardTicksPerMeter;
		double fr_dist = fr_tick / forwardTicksPerMeter;
		double bl_dist = bl_tick / forwardTicksPerMeter;
		double br_dist = br_tick / forwardTicksPerMeter;
		return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
	}

	public double getStrafeDistance() {
		double fl_tick = fl.getCurrentPosition() - fl_startPos;
		double fr_tick = fr.getCurrentPosition() - fr_startPos;
		double bl_tick = bl.getCurrentPosition() - bl_startPos;
		double br_tick = br.getCurrentPosition() - br_startPos;

		double flDist = fl_tick / strafeTicksPerMeter;
		double frDist = fr_tick / strafeTicksPerMeter;
		double blDist = bl_tick / strafeTicksPerMeter;
		double brDist = br_tick / strafeTicksPerMeter;
		return (-blDist + brDist - frDist + flDist) / 4;
	}

	public double getPosX() {
		return getAbsolutesPosX() + startingPosition.x;
	}

	public double getAbsolutesPosX() {
		double fl_tick = fl.getCurrentPosition();
		double fr_tick = fr.getCurrentPosition();
		double bl_tick = bl.getCurrentPosition();
		double br_tick = br.getCurrentPosition();

		double flDist = fl_tick / strafeTicksPerMeter;
		double frDist = fr_tick / strafeTicksPerMeter;
		double blDist = bl_tick / strafeTicksPerMeter;
		double brDist = br_tick / strafeTicksPerMeter;
		return (-blDist + brDist - frDist + flDist) / 4;
	}

	public void printWheelsPosition() {
		opMode.telemetry.addData("fl", fl.getCurrentPosition());
		opMode.telemetry.addData("fr", fr.getCurrentPosition());
		opMode.telemetry.addData("bl", bl.getCurrentPosition());
		opMode.telemetry.addData("br", br.getCurrentPosition());

		opMode.telemetry.addData("average", (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4);

		opMode.telemetry.addData("forwardDist", getForwardDistance());
		opMode.telemetry.addData("strafeDist", getStrafeDistance());
	}

	public void resetPosition() {
		fl_startPos = fl.getCurrentPosition();
		fr_startPos = fr.getCurrentPosition();
		bl_startPos = bl.getCurrentPosition();
		br_startPos = br.getCurrentPosition();
		opMode.telemetry.addData("RESET POSITION !!!!!!", 0);
	}


	public void turn(double deg, double power) {
		double targetAngle = getHeading() + deg; // zeroAngle
		turnTo(targetAngle, power);
	}

	public void turnTo(double targetAngle, double targetPower) {
		double delta = getDeltaHeading(targetAngle);
		double s = (delta < 0) ? -1 : 1;
		while ((delta * s > 0) && opMode.opModeIsActive()) {

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
		opMode.telemetry.addData("goto x", x);
		opMode.telemetry.addData("goto y", y);
		opMode.telemetry.update();

		drive(deltaY, deltaX, targetPower, targetHeading);
	}

	public void drive(double forward, double sideward, double targetPower, double targetAngle) {
		drive(forward, sideward, targetPower, targetAngle, true);
	}

	public void drive(double forward, double sideward, double targetPower, double targetAngle, boolean fieldOriented) {
		double sf = (forward < 0) ? -1 : 1;
		double ss = (sideward < 0) ? -1 : 1;
		double c = Math.sqrt(sideward * sideward + forward * forward);
		double RVf = forward / c;
		double RVs = sideward / c;
		final double minPower = 0.2;

		resetPosition();
		timer.reset();

		while (opMode.opModeIsActive() && (RVf != 0) ||  (RVs != 0)) {

			if (getForwardDistance() * sf > forward * sf - 0.02) {
				RVf = 0;
			}
			if (getStrafeDistance() * ss > sideward * ss - 0.02) {
				RVs = 0;
			}
			double power = targetPower ;

			double deltaForward = forward - getForwardDistance();
			double deltaStrafe  = sideward - getStrafeDistance();

			double deltaC = Math.sqrt(deltaStrafe * deltaStrafe + deltaForward * deltaForward);
			double lengthC = c - deltaC ;

			double acclGain = 2;
			double acclPower = lengthC * acclGain +  minPower;

			if (acclPower + 0.2 < power ) {
				power = acclPower;
			}

			double breakgain = 0.9;
			double breakPower = deltaC * breakgain + minPower;

			if (breakPower < power)  {
				power = breakPower;
			}

			double err = getDeltaHeading(targetAngle);
			double gain = 0.040;
			double correction = gain * err;
			double Vf = RVf * power;
			double Vs = RVs * power;

			setPowerOriented(Vf, Vs, correction, fieldOriented);

			opMode.telemetry.addData("time", timer.milliseconds());
			//position Telemetry:
			opMode.telemetry.addData("x position:", getAbsolutesPosX());
			opMode.telemetry.addData("y position:", getAbsolutesPosY());


			opMode.telemetry.addData("delta forward:", deltaForward);
			opMode.telemetry.addData("speed forward:", Vf);
//			opMode.telemetry.addData("delta strafe:", deltaStrafe);
			opMode.telemetry.addData("speed strafe:", Vs);
			opMode.telemetry.addData("power:", power);
			opMode.telemetry.addData("current pos", getForwardDistance());

			opMode.telemetry.update();
		}
		stopPower();
	}

}
