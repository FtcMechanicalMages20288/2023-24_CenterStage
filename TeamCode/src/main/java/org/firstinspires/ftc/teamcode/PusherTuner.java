package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pusher+RampSenseTuner", group = "TeleOp")




public class PusherTuner extends OpMode{
//drop 0.2
//Intake 0.5

    double tempR = 0;
    double tempL = 0;
    double tempHold = 0;

    private Servo PixelPusher;

    private NormalizedColorSensor RampSensor;
    @Override
    public void init() {
        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");
        RampSensor = hardwareMap.get(NormalizedColorSensor.class, "RampSensor");



        //PixelPusher.setPosition(0.5);
        //tempL = 0.5;



    }

    @Override
    public void loop() {
        if(gamepad2.a){

            PixelPusher.setPosition(tempL + 0.05);
            tempL =  PixelPusher.getPosition();
            telemetry.addData("Servo PosL", PixelPusher.getPosition());


            sleep(100);

        }
        if(gamepad2.b){

            PixelPusher.setPosition(tempL - 0.05);
            tempL =  PixelPusher.getPosition();
            telemetry.addData("Servo PosL", PixelPusher.getPosition());


            sleep(100);

        }
        telemetry.addData("Ramp Sensor: " , ((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM));
        telemetry.update();


    }
}



