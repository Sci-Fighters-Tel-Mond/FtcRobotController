package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GameClass {

    private LinearOpMode opMode;

    private DcMotorEx shooter = null;
    private DcMotorEx lifter = null;
    private DcMotorEx intake = null;

    private DcMotorEx wobble = null;
    private Servo wobbleGrabber1 = null;
    private Servo wobbleGrabber2 = null;
    private DigitalChannel wobbleLimiter = null;
    private DigitalChannel lifterLimiter = null;

    private Servo ringMover = null; // 1 - inside, 0 - outside

    private Toggle superState; // true - shooterPosition
    private Toggle shooterState;
    private Toggle intakeState;
    private Toggle wobbleGrabberState;
    private Toggle testToggle = new Toggle();

    private boolean updateLifterState = false;

    private int lifterupTargetPosition = 120;

    public GameClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        //region get from hw
        shooter = hw.get(DcMotorEx.class, "shooter");
        lifter = hw.get(DcMotorEx.class, "lifter");
        intake = hw.get(DcMotorEx.class, "collector");

        wobble = hw.get(DcMotorEx.class, "wobble");
        wobbleGrabber1 = hw.get(Servo.class, "wobble_grabber1");
        wobbleGrabber2 = hw.get(Servo.class, "wobble_grabber2");
        wobbleLimiter = hw.get(DigitalChannel.class, "wobble_limiter");
        lifterLimiter = hw.get(DigitalChannel.class, "shooter_limiter");

        ringMover = hw.get(Servo.class, "ring_mover");
        //endregion get from hw

        //region setDirection
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        wobble.setDirection(DcMotorEx.Direction.REVERSE);
        //endregion setDirection

        //region encoders
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //endregion encoders

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        superState = new Toggle();
        shooterState = new Toggle();
        wobbleGrabberState = new Toggle();
        intakeState = new Toggle();

        ringMover.setPosition(1);
    }

    public void setShooterPosition(boolean active) {
        if (active) {
            setIntake(false);
            setShooter(true);
            lifterUp(true); // up
        } else {
            setShooter(false); // stop shooter
            lifterUp(false); // down
            updateLifterState = true;
        }
    }

    public void update() {
        opMode.telemetry.addData("lifter pos", lifter.getCurrentPosition());

        if (updateLifterState == true) {
            if (getLifterLimiter()) {
                setIntake(true);
                updateLifterState = false;
                superState.set(false);
            }
         }

        if (lifter.getCurrentPosition() > lifterupTargetPosition - 10) {
            superState.set(true);
        }
    }

    public void lifterUp(boolean active) {
        //up
        int targetPosition;
        if (active) {
            targetPosition = lifterupTargetPosition;
        } else {
            targetPosition = 0;
        }
        lifter.setTargetPosition(targetPosition);
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.5);
    }

    public void lifterTest(double pow) {
        opMode.telemetry.addData("power", pow);

        testToggle.update(Math.abs(pow) > 0.2);
        if (testToggle.isClicked()) {
            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (testToggle.isPressed()) {
            lifter.setPower(pow);
        } else if (testToggle.isChanged()) {
            lifter.setPower(0);
        }
    }

    public void lifterRestart() {
        if (getLifterLimiter() == false) {
            lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lifter.setPower(-0.3);
            ElapsedTime timer = new ElapsedTime();
            while (getLifterLimiter() == false) {
                if (timer.milliseconds() >= 4000) break;
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


    private void setShooter(boolean active) {
        shooterState.set(active);
        shooter.setPower(active ? 0.8 : 0);
    }

    private void toggleShooter() {
        boolean state = shooterState.toggle();
        setShooter(state);
    }


    private void setIntake(boolean active) {
        intakeState.set(active);
        intake.setPower(active ? 1 : 0);
    }

    private void toggleCollector() {
        boolean state = intakeState.toggle();
        setIntake(state);
    }


    public void setWobble(double pow) {
        if (getWobbleLimiter()) {
            pow = Math.max(pow, 0);
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
        if (superState.getState()) {
            ringMover.setPosition(amt);
        }
    }

    public void shoot() {
        if (superState.getState()) {
            setRingMover(0);
            opMode.sleep(300);
            setRingMover(1);
        }
    }

    public void setIntakePower(double v) {
        intake.setPower(v);
    }

    public void stopAll() {
        shooter.setPower(0);
        intake.setPower(0);
        wobble.setPower(0);
        lifter.setPower(0);
    }
}
