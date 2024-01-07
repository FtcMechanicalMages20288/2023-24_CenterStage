package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Rumble Testing", group ="OpMode")
public class RumbleTester extends LinearOpMode {
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    @Override
    public void runOpMode() {
        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 750)  //  Rumble right motor 100% for 500 mSec
                .build();

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                sleep(1000);
                gamepad1.stopRumble();
            }
        }
    }
}
