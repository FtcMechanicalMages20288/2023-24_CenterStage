package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Color Dectection", group = "TeleOp")

public class ColorDetect extends OpMode {

    ColorSensor ColorFront, Color;
    @Override
    public void init() {
        ColorFront = hardwareMap.get(ColorSensor.class, "Color2");
        Color = hardwareMap.get(ColorSensor.class, "Color");

        telemetry.addData("ColorFront Red: ", ColorFront.red());
        telemetry.addData("ColorFront Blue: ", ColorFront.blue());
        telemetry.addData("ColorFront Green: ", ColorFront.green());
        telemetry.update();
    }

    @Override
    public void loop() {


        //Yellow
      if( (200 < ColorFront.red() && ColorFront.red() < 300 ) && ( 100 < ColorFront.blue()  ) && ( ColorFront.blue() < 200 )  && (360 <  ColorFront.green())  &&(  ColorFront.green() < 480 ) ){
          telemetry.addData("Color: " ,"Yellow");
        }
        //White
        if( (420 < ColorFront.red() && ColorFront.red() < 520 ) && ( 650 < ColorFront.blue()  ) && ( ColorFront.blue() < 768 )  && (720 <  ColorFront.green())  &&(  ColorFront.green() < 820 ) ){
            telemetry.addData("Color: " ,"White");
        }
        //Green
        if( (20 < ColorFront.red() && ColorFront.red() < 130 ) && ( 100 < ColorFront.blue()  ) && ( ColorFront.blue() < 200 )  && (150 <  ColorFront.green())  &&(  ColorFront.green() < 240 ) ){
            telemetry.addData("Color: " ,"Green");
        }
        //Purple
        if( (200 < ColorFront.red() && ColorFront.red() < 300 ) && ( 430 < ColorFront.blue()  ) && ( ColorFront.blue() < 530 )  && (310 <  ColorFront.green())  &&(  ColorFront.green() < 400 ) ){
            telemetry.addData("Color: " ,"Purple");
        }

        telemetry.update();



    }
}