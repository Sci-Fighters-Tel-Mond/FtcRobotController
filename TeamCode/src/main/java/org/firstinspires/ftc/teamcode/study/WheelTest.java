package org.firstinspires.ftc.teamcode.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group = "Cobalt")
//@Disabled
public class WheelTest extends LinearOpMode {
	private DcMotorEx fl = null;
	private DcMotorEx fr = null;
	private DcMotorEx bl = null;
	private DcMotorEx br = null;

	@Override
	public void runOpMode() {
		fl = hardwareMap.get(DcMotorEx.class, "fl");
		fr = hardwareMap.get(DcMotorEx.class, "fr");
		bl = hardwareMap.get(DcMotorEx.class, "bl");
		br = hardwareMap.get(DcMotorEx.class, "br");

		fl.setDirection(DcMotorEx.Direction.REVERSE);
		fr.setDirection(DcMotorEx.Direction.FORWARD);
		bl.setDirection(DcMotorEx.Direction.REVERSE);
		br.setDirection(DcMotorEx.Direction.FORWARD);

		fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


//        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		telemetry.addData("Status", "Initialized");
		telemetry.update();


		waitForStart();
		while (opModeIsActive()) {
			fl.setPower(gamepad1.left_stick_x);
			fr.setPower(gamepad1.right_stick_x);

			bl.setPower(-gamepad1.left_stick_y);
			br.setPower(-gamepad1.right_stick_y);

			telemetry.addData("front left:", fl.getCurrentPosition());
			telemetry.addData("front right:", fr.getCurrentPosition());
			telemetry.addData("back left:", bl.getCurrentPosition());
			telemetry.addData("back right:", br.getCurrentPosition());
			telemetry.update();
		}
	}
}
