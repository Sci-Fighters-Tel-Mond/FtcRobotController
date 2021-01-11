/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveClass;
import org.firstinspires.ftc.teamcode.util.GameClass;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(group="Linear Opmode")
//@Disabled
public class Avokatze extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DriveClass drive = new DriveClass(this).useBrake();
    private GameClass game = new GameClass(this);

    private Toggle lDpad = new Toggle();
    private Toggle rDpad = new Toggle();

    private Toggle shooter = new Toggle();
    private Toggle wobbleGrabber = new Toggle();
    private Toggle collector = new Toggle();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap);
        game.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lDpad.update(gamepad1.dpad_left );
            rDpad.update(gamepad1.dpad_right);

            shooter.update(gamepad1.y);
            wobbleGrabber.update(gamepad1.left_bumper);
            collector.update(gamepad1.x);


            double boost  = gamepad1.left_trigger * 0.4 + 0.6;

            double y  = -gamepad1.left_stick_y * boost;
            double x = gamepad1.left_stick_x * boost;
            double turn     = gamepad1.right_stick_x * boost;
            boolean fieldOriented = true; // gamepad1.left_bumper != true;

            drive.setPowerOriented(y, x, turn, fieldOriented);

            if (shooter.isClicked()) {
                game.toggleShooter();
            }
            if (wobbleGrabber.isClicked()) {
                game.toggleWobbleGrabber();
            }
            if (collector.isClicked()) {
                game.toggleCollector();
            }


            if (gamepad1.dpad_up) {
                game.setWobble(0.6);
            } else if (gamepad1.dpad_down) {
                game.setWobble(-0.6);
            } else {
                game.setWobble(0);
            }

            if (gamepad1.a) {
                game.setRingMover(0);
            } else if (gamepad1.b) {
                game.setRingMover(1);
            }

            if (gamepad1.right_trigger > 0.4) {
                game.setRingMover(1);
                sleep(200);
                game.setRingMover(0);
            }
        }
    }
}
