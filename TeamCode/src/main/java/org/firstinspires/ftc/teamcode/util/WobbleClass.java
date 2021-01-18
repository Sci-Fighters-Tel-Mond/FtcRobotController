package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleClass {
	private DcMotorEx wobble = null;
	private Servo wobbleGrabber1 = null;
	private Servo wobbleGrabber2 = null;
	private DigitalChannel wobbleLimiter = null;

	private Toggle wobbleGrabberState;

	public void init(HardwareMap hw) {
		//region get from hw
		wobble = hw.get(DcMotorEx.class, "wobble");
		wobbleGrabber1 = hw.get(Servo.class, "wobble_grabber1");
		wobbleGrabber2 = hw.get(Servo.class, "wobble_grabber2");
		wobbleLimiter = hw.get(DigitalChannel.class, "wobble_limiter");
		//endregion get from hw

		//region encoders
		wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		//endregion encoders

		wobbleGrabberState = new Toggle();
	}

	public void setWobble(double pow) {
		if (getWobbleLimiter()) {
			pow = Math.min(pow, 0);
		}
		wobble.setPower(pow);
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

	public boolean getWobbleLimiter() {
		return !wobbleLimiter.getState();
	}
}
