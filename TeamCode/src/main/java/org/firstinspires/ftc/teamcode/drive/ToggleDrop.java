package org.firstinspires.ftc.teamcode.drive;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Test Toggling Servo
 */
@Config
@TeleOp(group = "ToggleDrop")
public class ToggleDrop extends OpMode {

    Servo BucketR, BucketL, BucketHold;

    boolean lastMovement = false, currMovement = false;
    boolean downPosition = true;
    @Override
    public void init() {
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");

    }

    @Override
    public void loop() {



        lastMovement = currMovement;
        currMovement = gamepad1.a;

/*        ToggleDrop Thread = new ToggleDrop();
        Thread.start();*/

        if(currMovement && !lastMovement){
            downPosition = !downPosition;
            if(downPosition){
                BucketHold.setPosition(.4); //Move Down
            }else{
                BucketHold.setPosition(.5); //Move Up
                sleep(75);
                BucketHold.setPosition(.4);
            }
        }


    }
}
