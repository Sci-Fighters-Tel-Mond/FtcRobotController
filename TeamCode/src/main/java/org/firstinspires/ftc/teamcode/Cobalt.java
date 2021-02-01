package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group = "Cobalt")
//@Disabled
public class Cobalt extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT).useEncoders().useBrake();
    private GameClass game = new GameClass(this);

    private Toggle wobbleGrabber = new Toggle();
    private Toggle reverseIntake = new Toggle();

//    private Toggle shoot = new Toggle();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);
        game.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        game.lifterRestart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            wobbleGrabber.update(gamepad1.left_bumper);
            reverseIntake.update(gamepad1.dpad_left);

//            shoot.update(gamepad1.right_bumper);
            boolean ringFire = gamepad1.right_bumper;

            boolean fieldOriented = !gamepad1.left_bumper;
            double boost = gamepad1.right_trigger * 0.6 + 0.4;

            double y = -gamepad1.left_stick_y * boost;
            double x = gamepad1.left_stick_x * boost;
            double turn = gamepad1.right_stick_x * boost;

            boolean armShooter = gamepad1.x;
            boolean grabberClose = gamepad1.y;
            boolean grabberOpen = gamepad1.b;
            boolean stopAll = gamepad1.a;

            boolean resetOrientation = gamepad1.start;


            boolean wobbleForward = gamepad1.dpad_up;
            boolean wobbleBackWard = gamepad1.dpad_down;
            boolean intake = gamepad1.dpad_right;



            drive.setPowerOriented(y, x, turn, fieldOriented);

            if (resetOrientation) {
                drive.resetOrientation();
                drive.resetPosition();
            }

            if (wobbleForward) {
                game.setWobble(0.6);
            } else if (wobbleBackWard) {
                game.setWobble(-0.6);
            } else {
                game.setWobble(0);
            }

            if (ringFire) {
                game.setRingMover(0);
            } else {
                game.setRingMover(1);
            }
//            if (shoot.isClicked()) {
//                game.shoot();
//            }


            if (grabberOpen) {
                game.setWobbleGrabber(true);
            }

            if (grabberClose) {
                game.setWobbleGrabber(false);
            }

            if (armShooter) {
                game.setShooterPosition(true);
            }

            if (intake) {
                game.setShooterPosition(false);
            }

            if (reverseIntake.getState()) {

            }

            if (reverseIntake.isClicked()) {
                game.setIntakePower(-1);
            }

            if (reverseIntake.isPressed()) {
                game.setIntakePower(-1);
            } else {
                if (reverseIntake.isChanged()) {
                    game.setIntakePower(0);
                }
            }

            if (stopAll) {
                game.stopAll();
            }

            game.lifterTest(-gamepad1.right_stick_y);

            game.update();
            telemetry.update();
        }
    }
}
