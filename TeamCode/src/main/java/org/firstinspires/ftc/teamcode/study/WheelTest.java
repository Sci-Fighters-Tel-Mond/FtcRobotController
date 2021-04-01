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
    // Declare OpMode members.
    volatile private DcMotorEx fl = null;
    volatile private DcMotorEx fr = null;
    volatile private DcMotorEx bl = null;
    volatile private DcMotorEx br = null;



    public void init(HardwareMap hw) {
        //region get from hw
        fl = hw.get(DcMotorEx.class, "fl");
        fr = hw.get(DcMotorEx.class, "fr");
        bl = hw.get(DcMotorEx.class, "bl");
        br = hw.get(DcMotorEx.class, "br");
        //endregion get from hw

        //region setDirection
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);
        //endregion setDirection

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boolean useEncoders = true;

        //region setMode
        if (useEncoders) {
            fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
        //endregion setMode

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean back = gamepad1.right_bumper;

            double leftRight = -gamepad1.right_stick_y;



            if (back) {
                br.setPower(leftRight);
                bl.setPower(leftRight);
            }

            else{
                fl.setPower(leftRight);
                fr.setPower(leftRight);
            }


            telemetry.addData("front left:", fl.getCurrentPosition());
            telemetry.addData("front right:", fr.getCurrentPosition());
            telemetry.addData("back left:", bl.getCurrentPosition());
            telemetry.addData("back right:", br.getCurrentPosition());
            telemetry.update();
        }
    }
}
