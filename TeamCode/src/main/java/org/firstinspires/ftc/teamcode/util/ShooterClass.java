package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterClass {

    private LinearOpMode opMode;

    private DcMotorEx shooter = null;
    private DcMotorEx wobble = null;
    private DcMotorEx lifter = null;

    public ShooterClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        //region get from hw
        shooter = hw.get(DcMotorEx.class, "shooter");
        wobble = hw.get(DcMotorEx.class, "wobble");
        lifter = hw.get(DcMotorEx.class, "lifter");

        //region setDirection
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        wobble.setDirection(DcMotorEx.Direction.FORWARD);
        lifter.setDirection(DcMotorEx.Direction.REVERSE);
        //endregion setDirection

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setShooterSpeed(double speed) {
        shooter.setPower(speed);
    }
    public double getShooterSpeed() {
        double speed = shooter.getVelocity();
        return speed;
    }
}
