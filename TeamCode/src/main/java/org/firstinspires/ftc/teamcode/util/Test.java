package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Cobalt")
//@Disabled
public class Test extends LinearOpMode {
    final double tile = 0.6;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Location startingPosition = new Location(-1.75*tile,0*tile);
    private DriveClass drive = new DriveClass(this, DriveClass.ROBOT.COBALT, startingPosition).useEncoders().useBrake();
    private GameClass game = new GameClass(this);

    private Toggle reverseIntake = new Toggle();
    private Toggle lifterUp = new Toggle();
    private Toggle lifterDown = new Toggle();
    private Toggle shootHeading = new Toggle();
    private Toggle ringFire = new Toggle();
    private Toggle turningToggle = new Toggle();

    private double targetHeading = 0;

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

        drive.resetOrientation(90); //default blue

        runtime.reset();

        int turningCount = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean armShooter = gamepad1.x && !gamepad1.start; // up armShooter
            boolean grabberOpen = gamepad1.y && !gamepad1.start; // open wobble grabbers.
            boolean grabberClose = gamepad1.b;
            boolean stopAll = gamepad1.a;

            boolean intake = gamepad1.dpad_right; // down armShooter

            reverseIntake.update(gamepad1.dpad_left);
            lifterUp.update(gamepad1.dpad_up);
            lifterDown.update(gamepad1.dpad_down);
            shootHeading.update(gamepad1.back);
            ringFire.update(gamepad1.right_bumper);


            boolean resetOrientation = gamepad1.start;

            if (resetOrientation) {
                if (gamepad1.x){
                    drive.resetOrientation(90);
                }
                if(gamepad1.y){
                    drive.resetOrientation(-90);
                }
                drive.resetPosition();
                targetHeading = drive.getHeading();
                continue;
            }

            boolean fieldOriented = !gamepad1.left_bumper;
            double boost = gamepad1.right_trigger * 0.6 + 0.4;

            double y = -gamepad1.left_stick_y * boost;
            double x = gamepad1.left_stick_x * boost;
            double turn = gamepad1.right_stick_x * boost;

            turningToggle.update(Math.abs(turn)>0.05);

            if (turningToggle.isReleased()){
                turningCount = 100;
            }
            if (!turningToggle.isPressed()){
                turningCount--;
            }

            if(turningCount == 0){
                targetHeading = drive.getHeading();

            }

            if (! turningToggle.isPressed() && turningCount < 0){
                double delta = drive.getDeltaHeading(targetHeading);
                double gain = 0.05;
                turn = delta * gain;
            }

            drive.setPowerOriented(y, x, turn, fieldOriented);

            if (shootHeading.isClicked()) {
                drive.turnTo(3, 1);
            }

            if (ringFire.isClicked()) {
                game.setRingMover(0);
                sleep(300);
                game.setRingMover(1);
            }

            if (lifterDown.isClicked()) {
                game.lifterMove(-10);

            }

            if (lifterUp.isClicked()) {
                game.lifterMove(10);
            }


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
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Delta", drive.getDeltaHeading(targetHeading));

            game.update();
            telemetry.update();
        }
    }
}
