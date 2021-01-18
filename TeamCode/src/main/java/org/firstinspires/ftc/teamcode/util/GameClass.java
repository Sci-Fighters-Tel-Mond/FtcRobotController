package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GameClass {

    private LinearOpMode opMode;

    private DcMotorEx shooter = null;
    private DcMotorEx lifter = null;
    private DcMotorEx collector = null;

    private DcMotorEx wobble = null;
    private Servo wobbleGrabber1 = null;
    private Servo wobbleGrabber2 = null;
    private DigitalChannel wobbleLimiter = null;

    private Servo ringMover = null; // 1 - inside, 0 - outside

    private Toggle shooterState;
    private Toggle collectorState;
    private Toggle wobbleGrabberState;

    public GameClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        //region get from hw
        shooter = hw.get(DcMotorEx.class, "shooter");
        lifter = hw.get(DcMotorEx.class, "lifter");
        collector = hw.get(DcMotorEx.class, "collector");

        wobble = hw.get(DcMotorEx.class, "wobble");
        wobbleGrabber1 = hw.get(Servo.class, "wobble_grabber1");
        wobbleGrabber2 = hw.get(Servo.class, "wobble_grabber2");
        wobbleLimiter = hw.get(DigitalChannel.class, "wobble_limiter");

        ringMover = hw.get(Servo.class, "ring_mover");
        //endregion get from hw

        //region setDirection
        lifter.setDirection(DcMotorEx.Direction.REVERSE);
        collector.setDirection(DcMotorEx.Direction.REVERSE);

        wobble.setDirection(DcMotorEx.Direction.FORWARD);
        //endregion setDirection

        //region encoders
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //endregion encoders

        shooterState = new Toggle();
        wobbleGrabberState = new Toggle();
        collectorState = new Toggle();
    }

    public void setShooter(boolean active) {
        shooterState.set(active);
        shooter.setPower(active ? 0.8 : 0);
    }

    public void toggleShooter() {
        boolean state = shooterState.toggle();
        setShooter(state);
    }


    public void setCollector(boolean active) {
        collectorState.set(active);
        collector.setPower(active ? 1 : 0);
    }

    public void toggleCollector() {
        boolean state = collectorState.toggle();
        setCollector(state);
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


    public void setRingMover(double amt) {
        ringMover.setPosition(amt);
    }
}
