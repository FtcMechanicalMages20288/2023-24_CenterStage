package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BucketTuner", group = "TeleOp")


public class BucketTuner extends OpMode {
//drop 0.2
//Intake 0.5

    double tempR = 0;
    double tempL = 0;
    double tempHold = 0.5;

    private DcMotor SlideR, SlideL;

    private Servo BucketHold, BucketR, BucketL, HangR, HangL;

    private TouchSensor LimitSwitch;

    private NormalizedColorSensor Color, ColorFront;

    @Override
    public void init() {
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        // BucketR.setDirection(Servo.Direction.REVERSE);

        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");

        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");

        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");

        Color = hardwareMap.get(NormalizedColorSensor.class, "Color");
        ColorFront = hardwareMap.get(NormalizedColorSensor.class, "Color2");


      /*  BucketL.setPosition(0.5);
        BucketR.setPosition(0.4); */
        //  tempR = 0.46;
        //  tempL = 0.5;
       /* HangR.setPosition(0.5);
        HangL.setPosition(0.5);
        HangR.setDirection((Servo.Direction.REVERSE));
        BucketR.setDirection(Servo.Direction.REVERSE); */
        // BucketHold.setPosition(0.1);


        BucketR.setPosition(0.62);
        BucketL.setPosition(0.4);
    }

    @Override
    public void loop() {
        telemetry.addData("BucketR: ", BucketR.getPosition());
        telemetry.addData("BucketL:  ", BucketL.getPosition());
        telemetry.addData("Color: ", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
        telemetry.addData("ColorFront: ", ((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM));

        if (gamepad2.a) {
            tempR = BucketR.getPosition();
            tempL = BucketL.getPosition();
            BucketR.setPosition(tempR + 0.05);
            //BucketL.setPosition(tempL + 0.05);
            telemetry.addData("Servo PosL", BucketL.getPosition());
            telemetry.addData("Servo PosR", BucketR.getPosition());
            telemetry.update();
            sleep(1000);

        }

        if (gamepad2.b) {
            tempR = BucketR.getPosition();
            tempL = BucketL.getPosition();
            BucketR.setPosition(tempR - 0.05);
            //BucketL.setPosition(tempL - 0.05);
            telemetry.addData("Servo PosL", BucketL.getPosition());
            telemetry.addData("Servo PosR", BucketR.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if (gamepad2.y) {
            BucketHold.setPosition(tempHold - 0.05);
            tempHold = BucketHold.getPosition();
            telemetry.addData("Servo Pos Hold", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }
        if (gamepad2.x) {
            BucketHold.setPosition(tempHold + 0.05);
            tempHold = BucketHold.getPosition();
            telemetry.addData("Servo Pos Hold", BucketHold.getPosition());
            telemetry.update();
            sleep(1000);
        }

        if (!LimitSwitch.isPressed() && !gamepad1.b) {
            if (gamepad1.a) { //down
                SlideL.setPower(-.6);
                SlideR.setPower(.6);
                telemetry.addData("If statement for button A", BucketHold.getPosition());
            }
            else{
                SlideL.setPower(0.08);
                SlideR.setPower(-0.08);
            }

        } else if(LimitSwitch.isPressed() && !gamepad1.b){
                SlideL.setPower(0);
                SlideR.setPower(0);

        } else  if (gamepad1.b) {
            SlideL.setPower(.6);
            SlideR.setPower(-.6);
        }

       /* else if(!gamepad1.b && !gamepad1.a && LimitSwitch.isPressed()) {
        SlideL.setPower(0.1);
        SlideR.setPower(-0.1);&-


    }*/




    }
}



