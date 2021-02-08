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

    private Toggle superState = new Toggle();// true - shooterPosition
    private Toggle shooterState = new Toggle();
    private Toggle intakeState = new Toggle();
    private Toggle wobbleGrabberState = new Toggle();
    private Toggle testToggle = new Toggle();

    private boolean lifterDownRequest = false;
    private boolean lifterUpRequest = false;



    private int lifterupTargetPosition = 120;

    ElapsedTime timer = new ElapsedTime();


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

    public void setShooterPosition(boolean active) {
        if (active) {
            setIntake(false);
            setShooter(true);
            lifterUpDown(true); // up
        } else {
            setShooter(false); // stop shooter
            lifterUpDown(false); // down
            lifterDownRequest = true;
        }
    }

    public void update() {
        opMode.telemetry.addData("lifter pos", lifter.getCurrentPosition());


        if (lifterDownRequest) {
            if (getLifterLimiter() || timer.milliseconds() > 1000) {
                setIntake(true);
                lifterDownRequest = false;
                superState.set(false);
                lifter.setPower(0);
            }
         }

        if (lifter.getCurrentPosition() > lifterupTargetPosition - 10) {
            superState.set(true);
        }

        if (lifterUpRequest){
            if (timer.milliseconds() > 1000 ){
                lifter.setPower(0.1);
                lifterUpRequest = false;
                superState.set(true);
            }
            opMode.telemetry.addData("timer mls", timer.milliseconds());

        }
    }

    public void lifterUpDown(boolean isup) {
        if (isup) {
            lifter.setPower(1);
            lifterUpRequest = true;
            lifterDownRequest = false;
            timer.reset();
        } else {
            lifter.setPower(-0.5);
            lifterUpRequest = false;
            lifterDownRequest = true;
            timer.reset();
        }
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

    public void wobbleArmRestart(){
        wobbleArm.setPower(-0.6);
        ElapsedTime time = new ElapsedTime();
        while (getWobbleArmLimiter() == false){
           if ( time.milliseconds() > 2000) break;
        }
        wobbleArm.setPower(0);
        wobbleArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void lifterRestart() {
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
//        lifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        wobbleArm.setPower(0);
        lifter.setPower(0);
    }
}
