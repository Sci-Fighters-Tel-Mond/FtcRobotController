
package org.firstinspires.ftc.teamcode.study;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Toggle;


@TeleOp(name="Wobble Mover", group="Linear Opmode")
//@Disabled
public class Wobble extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //
    private DcMotor wobble_mover = null;
    private Servo wobble_grabber1 = null;
    private Servo wobble_grabber2 = null;
    private Toggle wobble_g1_state = new Toggle(false);
    private Toggle wobble_g2_state = new Toggle(false);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        wobble_mover  = hardwareMap.get(DcMotor.class, "wobble_mover");
        wobble_grabber1  = hardwareMap.get(Servo.class, "wobble_grabber1");
        wobble_grabber2  = hardwareMap.get(Servo.class, "wobble_grabber2");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            //wobble_mover:
            if(gamepad1.right_trigger > 0.2){
                wobble_mover.setDirection(DcMotor.Direction.FORWARD);
                wobble_mover.setPower(0.85);
            }

            if (gamepad1.right_trigger > 0.2){
                wobble_mover.setDirection(DcMotor.Direction.REVERSE);
                wobble_mover.setPower(0.85);
            }

            //wobble_grabber1:
            if (gamepad1.right_bumper){
                wobble_grabber1.setPosition(1);
                wobble_g1_state.update(true);

                wobble_grabber2.setPosition(0);
                wobble_g2_state.update(true);
            }



            //wobble_grabber2:

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("trigger:", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}
