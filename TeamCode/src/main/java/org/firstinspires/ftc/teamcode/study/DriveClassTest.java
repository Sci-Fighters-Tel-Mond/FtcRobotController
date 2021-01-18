package org.firstinspires.ftc.teamcode.study;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.DriveClass;


@TeleOp(group="Test")
//@Disabled
public class DriveClassTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DriveClass robot = new DriveClass(this, DriveClass.ROBOT.SCORPION)
                                        .useEncoders()
                                        .useBrake();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double boost = gamepad1.left_trigger * 0.5 + 0.5;
            double drive = -gamepad1.left_stick_y * boost;
            double turn = gamepad1.right_stick_x * boost;
            double strafe = gamepad1.left_stick_x * boost;

            robot.setPower(drive, turn, strafe);
        }
    }
}
