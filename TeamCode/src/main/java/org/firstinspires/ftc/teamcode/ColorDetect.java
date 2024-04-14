package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Color Dectection", group = "TeleOp")

public class ColorDetect extends OpMode {

    ColorSensor ColorFront, Color;

    NormalizedColorSensor ColorFrontSense, ColorSense;
    @Override
    public void init() {
        ColorFront = hardwareMap.get(ColorSensor.class, "Color2");
        Color = hardwareMap.get(ColorSensor.class, "Color");

        ColorFrontSense = hardwareMap.get(NormalizedColorSensor.class,"Color2");
        ColorSense = hardwareMap.get(NormalizedColorSensor.class, "Color");

        telemetry.addData("ColorFront Red: ", ColorFront.red());
        telemetry.addData("ColorFront Blue: ", ColorFront.blue());
        telemetry.addData("ColorFront Green: ", ColorFront.green());
        telemetry.update();
    }

    @Override
    public void loop() {


        telemetry.addData("FrontBlue: ", ColorFront.blue());
        telemetry.addData("FrontRed: ", ColorFront.red());
        telemetry.addData("FrontGreen: ", ColorFront.green());

        telemetry.addData("BackBlue: ", Color.blue());
        telemetry.addData("BackRed: ", Color.red());
        telemetry.addData("BackGreen: ", Color.green());


        telemetry.addData("Normalized Color Front: " , ColorFrontSense.getNormalizedColors().toColor());
        telemetry.addData("Normalized Color Back: ", ColorSense.getNormalizedColors().toColor());

        telemetry.update();




      /*  //Yellow
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

        telemetry.update();*/



    }
}