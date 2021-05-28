package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import com.qualcomm.hardware.bosch.BNO055IMU.AccelerationIntegrator;
//import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
//import com.qualcomm.robotcore.util.RobotLog;


public class IMU_Integrator implements BNO055IMU.AccelerationIntegrator {

	volatile private DcMotorEx fl = null;
	volatile private DcMotorEx fr = null;
	volatile private DcMotorEx bl = null;
	volatile private DcMotorEx br = null;

	private int fl_startPos = 0;
	private int fr_startPos = 0;
	private int bl_startPos = 0;
	private int br_startPos = 0;

	private double forwardTicksPerMeter;
	private double strafeTicksPerMeter;

	private double xPos = 0;
	private double yPos = 0;

	private BNO055IMU imu = null;
	BNO055IMU.Parameters parameters = null;

	Position position = new Position();
	Velocity velocity = new Velocity();
	Acceleration acceleration = null;

	public Position getPosition() {
		return this.position;
	}
	public Velocity getVelocity() {
		return this.velocity;
	}
	public Acceleration getAcceleration() {
		return this.acceleration;
	}

	public  double getX() { return xPos; }
	public  double getY() { return yPos; }

	IMU_Integrator(BNO055IMU imu, HardwareMap hw, double forwardTicksPerMeter, double strafeTicksPerMeter) {
		this.imu = imu;
		// Constructor
		fl = hw.get(DcMotorEx.class, "fl");
		fr = hw.get(DcMotorEx.class, "fr");
		bl = hw.get(DcMotorEx.class, "bl");
		br = hw.get(DcMotorEx.class, "br");

		this.forwardTicksPerMeter = forwardTicksPerMeter;
		this.strafeTicksPerMeter = strafeTicksPerMeter;
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

	public void resetPosition() {
		fl_startPos = fl.getCurrentPosition();
		fr_startPos = fr.getCurrentPosition();
		bl_startPos = bl.getCurrentPosition();
		br_startPos = br.getCurrentPosition();
	}

	public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
		this.parameters = parameters;
		this.position = initialPosition != null ? initialPosition : this.position;
		this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
		this.acceleration = null;

		resetPosition();

		xPos = initialPosition.x;
		yPos = initialPosition.y;
	}

	public double getHeading() {
		Orientation orientation = imu.getAngularOrientation();
		return -orientation.firstAngle;
	}

	public void update(Acceleration linearAcceleration) {

		double deltaF = getForwardDistance();
		double deltaS = getStrafeDistance();
		resetPosition();

		double a = -getHeading() / 180.0 * Math.PI;
		xPos += deltaS * Math.cos(a) - deltaF * Math.sin(a);
		yPos += deltaF * Math.cos(a) + deltaS * Math.sin(a);

		position.x = xPos;
		position.y = yPos;

//		if (linearAcceleration.acquisitionTime != 0L) {
//			if (this.acceleration != null) {
//				Acceleration accelPrev = this.acceleration;
//				Velocity velocityPrev = this.velocity;
//				this.acceleration = linearAcceleration;
//				if (accelPrev.acquisitionTime != 0L) {
//					Velocity deltaVelocity = NavUtil.meanIntegrate(this.acceleration, accelPrev);
//					this.velocity = NavUtil.plus(this.velocity, deltaVelocity);
//				}
//
//				if (velocityPrev.acquisitionTime != 0L) {
//					Position deltaPosition = NavUtil.meanIntegrate(this.velocity, velocityPrev);
//					this.position = NavUtil.plus(this.position, deltaPosition);
//				}
//
//				if (this.parameters != null && this.parameters.loggingEnabled) {
//					RobotLog.vv(this.parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", new Object[]{(double)(this.acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1.0E-9D, this.acceleration, this.velocity, this.position});
//				}
//			} else {
//				this.acceleration = linearAcceleration;
//			}
//		}

	}
}
