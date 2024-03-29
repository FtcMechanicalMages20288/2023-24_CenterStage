package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "BucketTuner", group = "TeleOp")


public class BucketTuner extends OpMode{
//drop 0.2
//Intake 0.5

    double tempR = 0;
    double tempL = 0;
    double tempHold = 0.5;

    private Servo BucketHold, BucketR, BucketL, HangR, HangL;
    @Override
    public void init() {
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketR.setDirection(Servo.Direction.REVERSE);

        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");


      /*  BucketL.setPosition(0.5);
        BucketR.setPosition(0.4); */
      //  tempR = 0.46;
      //  tempL = 0.5;
       /* HangR.setPosition(0.5);
        HangL.setPosition(0.5);
        HangR.setDirection((Servo.Direction.REVERSE));
        BucketR.setDirection(Servo.Direction.REVERSE); */
       // BucketHold.setPosition(0.1);

        BucketR.setPosition(0.5);
        BucketL.setPosition(0.3);


    }

    @Override
    public void loop() {
        if(gamepad2.a){
            BucketR.setPosition(tempR + 0.05);
            BucketL.setPosition(tempL + 0.05);
            tempR =  BucketR.getPosition();
            tempL =  BucketL.getPosition();
            telemetry.addData("Servo PosL", BucketL.getPosition());
            telemetry.addData("Servo PosR", BucketR.getPosition());
            telemetry.update();
           sleep(1000);

        }

        if(gamepad2.b) {
            BucketR.setPosition(tempR - 0.05);
            BucketL.setPosition(tempL - 0.05);
            tempR = BucketR.getPosition();
            tempL =  BucketL.getPosition();
            telemetry.addData("Servo PosL", BucketL.getPosition());
            telemetry.addData("Servo PosR", BucketR.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if(gamepad2.y){
            BucketHold.setPosition(tempHold - 0.05);
            tempHold =  BucketHold.getPosition();
            telemetry.addData("Servo Pos Hold", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if(gamepad2.x){
            BucketHold.setPosition(tempHold + 0.05);
            tempHold =  BucketHold.getPosition();
            telemetry.addData("Servo Pos Hold", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }

        if(gamepad1.a){
            BucketHold.setPosition(0.5);
        }
        if(gamepad1.b){
            BucketHold.setPosition(0.55);
        }
        if(gamepad1.x){
            BucketHold.setPosition(0.5);
            sleep(25);
            BucketHold.setPosition(0.65);

        }

    }
}



