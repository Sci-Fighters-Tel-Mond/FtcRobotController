package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Location;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group = "Cobalt")
//@Disabled
public class Cobalt extends LinearOpMode {
    final double tile = 0.6;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Location startingPosition = new Location(Location.LOCATION.BLUE_EXTERNAL_START_POSITION,-1.75*tile,0*tile);
    private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
    private GameClass game = new GameClass(this);

    private Toggle reverseIntake = new Toggle();
    private Toggle wobbleForward = new Toggle();
    private Toggle wobbleBackward = new Toggle();
    private Toggle shootHeading = new Toggle();
    private Toggle ringFire = new Toggle();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);
        game.init(hardwareMap);

        game.initLifterPosition();
        game.initWobbleArmPosition();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean fieldOriented = !gamepad1.left_bumper;
            double boost = gamepad1.right_trigger * 0.6 + 0.4;

            double y = -gamepad1.left_stick_y * boost;
            double x = gamepad1.left_stick_x * boost;
            double turn = gamepad1.right_stick_x * boost;

            boolean armShooter = gamepad1.x && !gamepad1.start; // up armShooter
            boolean grabberOpen = gamepad1.y && !gamepad1.start; // open wobble grabbers.
            boolean grabberClose = gamepad1.b;
            boolean stopAll = gamepad1.a;

            boolean resetOrientation = gamepad1.start;

            boolean intake = gamepad1.dpad_right; // down armShooter
            reverseIntake.update(gamepad1.dpad_left);
            wobbleForward.update(gamepad1.dpad_up);
            wobbleBackward.update(gamepad1.dpad_down);
            shootHeading.update(gamepad1.back);
            ringFire.update(gamepad1.right_bumper);

            drive.setPowerOriented(y, x, turn, fieldOriented);

            if (resetOrientation) {
                drive.resetOrientation(90);
                drive.resetPosition();
            }

            if (shootHeading.isClicked()) {
                drive.turnTo(3, 1);
            }

            if (ringFire.isClicked()) {
                game.setRingMover(0);
                sleep(300);
                game.setRingMover(1);
            }

            if (wobbleBackward.isClicked()) {
                game.wobbleArmGoTo(2850);
               // game.setWobbleArm(-0.6);
            } //else if (wobbleBackward.isReleased()){
                //game.setWobbleArm(0.0);
                //game.wobbleArmGoTo(3000);

           // }

            if (wobbleForward.isClicked()) {
                game.wobbleArmGoTo(6185);
                //game.setWobbleArm(0.6);
            }//} else if (wobbleForward.isReleased()) {
                //game.setWobbleArm(0);
            //}


            if (grabberOpen) {
                game.setWobbleGrabber(true);
            }

            if (grabberClose) {
                game.setWobbleGrabber(false);
            }

            if (armShooter) {
                game.setSuperPosition(true);
                telemetry.addData("X ", "IS PRESSED");

            }

            if (intake) {
                game.setSuperPosition(false);
            }

            if (reverseIntake.isPressed()) {
                game.setIntakePower(-1);
            } else {
                if (reverseIntake.isReleased()) {
                    game.setIntakePower(0);
                }
            }

            if (stopAll) {
                game.stopAll();
            }

            game.lifterTest(-gamepad1.right_stick_y);
            telemetry.addData("X Pos",drive.getPosX());
            telemetry.addData("Y Pos", drive.getPosY());
            telemetry.addData("Heading", drive.getHeading());



            game.update();
            telemetry.update();
        }
    }
}
