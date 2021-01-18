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

    private Servo ringMover = null; // 1 - inside, 0 - outside

    private Toggle shooterState;
    private Toggle collectorState;

    public GameClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        //region get from hw
        shooter = hw.get(DcMotorEx.class, "shooter");
        lifter = hw.get(DcMotorEx.class, "lifter");
        collector = hw.get(DcMotorEx.class, "collector");

        ringMover = hw.get(Servo.class, "ring_mover");
        //endregion get from hw

        //region setDirection
        lifter.setDirection(DcMotorEx.Direction.REVERSE);
        collector.setDirection(DcMotorEx.Direction.REVERSE);
        //endregion setDirection

        //region encoders
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //endregion encoders

        shooterState = new Toggle();
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


    public void setRingMover(double amt) {
        ringMover.setPosition(amt);
    }
}
