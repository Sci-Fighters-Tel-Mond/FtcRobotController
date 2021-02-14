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
    private DcMotorEx lifter = null;
    private DcMotorEx intake = null;

    private DcMotorEx wobbleArm = null;
    private Servo wobbleGrabber1 = null;
    private Servo wobbleGrabber2 = null;
    private DigitalChannel wobbleArmLimiter = null;
    private DigitalChannel lifterLimiter = null;

    private Servo ringMover = null; // 1 - inside, 0 - outside

    private Toggle supperState = new Toggle();// true - shooterPosition
    private Toggle shooterState = new Toggle();
    private Toggle intakeState = new Toggle();
    private Toggle wobbleGrabberState = new Toggle();
    private Toggle testLifterToggle = new Toggle();

    private boolean lifterDownRequest = false;
    private boolean lifterUpRequest = false;

    private int lifterUpTargetPosition = 120;

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

        wobbleArm.setDirection(DcMotorEx.Direction.REVERSE);
        //endregion setDirection

        //region encoders
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //endregion encoders

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ringMover.setPosition(1);
    }

    public void setSupperPosition(boolean goUp) {
        if (goUp) {
            setIntake(false);
            setShooterRoller(true);
            lifterUpDown(true); // up
        } else { // goDown
            setShooterRoller(false); // stop shooter
            lifterUpDown(false); // down
        }
    }

    public void lifterUpDown(boolean goUp) {
        if (goUp) {
            lifter.setPower(1);
            lifterUpRequest = true;
            lifterDownRequest = false;
        } else {
            lifter.setPower(-0.9);
            lifterUpRequest = false;
            lifterDownRequest = true;
        }
        timer.reset();
    }

    public void update() {
        opMode.telemetry.addData("Lifter pos", lifter.getCurrentPosition());

        if (lifterUpRequest){
            if (lifter.getCurrentPosition() > lifterUpTargetPosition - 10 || timer.milliseconds() > 1000 ){
                lifterUpRequest = false;
                lifter.setPower(0.1);
                supperState.set(true);
            }
            opMode.telemetry.addData("Lifter GO UP", timer.milliseconds());
        }

        if (lifterDownRequest) {
            if (getLifterLimiter() || (timer.milliseconds() > 1000)) {
                lifterDownRequest = false;
                setIntake(true);
                lifter.setPower(0);
                supperState.set(false);
            }
            opMode.telemetry.addData("Lifter GO Down", timer.milliseconds());
        }

        opMode.telemetry.addData("Supper State", supperState.getState());
    }

    public void lifterTest(double pow) {
        testLifterToggle.update(Math.abs(pow) > 0.2);
        if (testLifterToggle.isClicked()) {
            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (testLifterToggle.isPressed()) {
            opMode.telemetry.addData("TEST Lifter Power", pow);
            lifter.setPower(pow);
        } else if (testLifterToggle.isReleased()) {
            opMode.telemetry.addData("TEST Lifter Power", pow);
            lifter.setPower(0);
        }
    }

    public void wobbleArmInitPosition(){
        wobbleArm.setPower(-0.6);
        ElapsedTime time = new ElapsedTime();
        while (getWobbleArmLimiter() == false){
           if ( time.milliseconds() > 5000) break;
        }
        wobbleArm.setPower(0);
        wobbleArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void lifterInitPosition() {
        lifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (getLifterLimiter() == false) {
            lifter.setPower(-0.3);
            ElapsedTime timer = new ElapsedTime();
            while (getLifterLimiter() == false) {
                if (timer.milliseconds() > 2000) break;
            }
            lifter.setPower(0);
            opMode.sleep(1000);

        }
        lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        supperState.set(false);
    }


    public boolean getLifterLimiter() {
        return !lifterLimiter.getState();
    }


    private void setShooterRoller(boolean active) {
        shooterState.set(active);
        shooter.setPower(active ? 0.8 : 0);
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
        if (getWobbleArmLimiter()) {
            pow = Math.max(pow, 0);
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
        if (supperState.getState()) {
            ringMover.setPosition(amt);
        }
    }

    public void shoot() {
        if (supperState.getState()) {
            setRingMover(0);
            opMode.sleep(300);
            setRingMover(1);
        }
    }

    public void setIntakePower(double v) {
        intake.setPower(v);
    }

    public void stopAll() {
        supperState.set(false);
        setShooterRoller(false);
        setIntake(false);

        wobbleArm.setPower(0);
        lifter.setPower(0);
    }
}
