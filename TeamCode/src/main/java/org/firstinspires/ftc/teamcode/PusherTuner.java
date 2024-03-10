package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "PusherTuner", group = "TeleOp")
@Disabled



public class PusherTuner extends OpMode{
//drop 0.2
//Intake 0.5

    double tempR = 0;
    double tempL = 0;
    double tempHold = 0;

    private Servo PixelPusher;
    @Override
    public void init() {
        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");



        PixelPusher.setPosition(0.5);
        tempL = 0.5;



    }

    @Override
    public void loop() {
        if(gamepad2.a){

            PixelPusher.setPosition(tempL + 0.05);
            tempL =  PixelPusher.getPosition();
            telemetry.addData("Servo PosL", PixelPusher.getPosition());

            telemetry.update();
            sleep(100);

        }
        if(gamepad2.b){

            PixelPusher.setPosition(tempL - 0.05);
            tempL =  PixelPusher.getPosition();
            telemetry.addData("Servo PosL", PixelPusher.getPosition());

            telemetry.update();
            sleep(100);

        }


    }
}



