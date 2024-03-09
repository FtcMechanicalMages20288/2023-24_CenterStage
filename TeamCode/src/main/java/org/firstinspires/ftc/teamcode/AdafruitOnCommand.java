package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.w8wjb.ftc.AdafruitNeoDriver;

@TeleOp
(name = "AdafruitOnCommand", group = "TeleOp")
public class AdafruitOnCommand extends OpMode {

    private static final int NUM_PIXELS = 60;

    AdafruitNeoDriver neopixels;



    private int redStart = 0;
    private int hueGap = 0;

    @Override
    public void init() {

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neopixels");

        neopixels.setNumberOfPixels(NUM_PIXELS);

        hueGap = 360 / NUM_PIXELS;

    }

    @Override
    public void loop() {

        if (gamepad2.a) {


            colorOff();
        }
        if (gamepad2.b) {

            showGreen();

        }
        if (gamepad2.x) {

            Prowler();

        }
        if (gamepad2.y) {

            showPurple();

        }

    }
    private void showPurple(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = Color.HSVToColor(new float[] {278 , 89, 66 });
            colors[i] = color;
        }



        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void Prowler(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            if(i % 5 == 0) {
                //Green
                int color = Color.HSVToColor(new float[] {101 , 42, 66 });
                colors[i] = color;

            }
            else{
                //purple
                int color = Color.HSVToColor(new float[]{278, 89, 66});
                colors[i] = color;
            }
        }



        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGreen(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = Color.HSVToColor(new float[] {101 , 42, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGold(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = Color.HSVToColor(new float[] {247 , 98, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void colorOff(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = Color.HSVToColor(new float[] {0 , 0, 0 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }



}