package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "ServoTuner", group = "TeleOp")
public class BucketTuner extends OpMode{
//drop 0.2
//Intake 0.5

    double temp = 0;
    double tempHold = 0;

    private Servo BucketHold, BucketR, BucketL, HangR, HangL;
    @Override
    public void init() {
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");

        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");


        HangR.setPosition(0.5);
        HangL.setPosition(0.5);
        HangR.setDirection((Servo.Direction.REVERSE));
        BucketR.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if(gamepad2.a){
            HangR.setPosition(temp + 0.05);
            HangL.setPosition(temp + 0.05);
            temp =  BucketR.getPosition();
            telemetry.addData("Servo Pos", HangL.getPosition());
            telemetry.update();
           sleep(1000);

        }
        if(gamepad2.b) {
            HangR.setPosition(temp - 0.05);
            HangL.setPosition(temp - 0.05);
            temp =  BucketR.getPosition();
            telemetry.addData("Servo Pos", BucketR.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if(gamepad2.y){
            BucketHold.setPosition(tempHold - 0.05);
            tempHold =  BucketHold.getPosition();
            telemetry.addData("Servo Pos", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if(gamepad2.x){
            BucketHold.setPosition(tempHold + 0.05);
            tempHold =  BucketHold.getPosition();
            telemetry.addData("Servo Pos", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }

    }
}



