package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.w8wjb.ftc.AdafruitNeoDriver;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "MecanumState", group = "TeleOp")

public class MecanumLinear extends LinearOpMode {


    //DriveTrain Motors
    private  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone, HangR, HangL, Grabber;
    private CRServo IntakeRoller;







    //limitswitch
    private TouchSensor LimitSwitch;
    //BoxColorSensor
    private NormalizedColorSensor Color;

    long startTime = System.currentTimeMillis();


    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;

    int Pixels = 0;

    private static final int NUM_PIXELS = 60;

    AdafruitNeoDriver neopixels;


    boolean Rumbled = false;

    boolean yPressed = false;

    boolean boardAdjust = false;

    boolean pixelGap = false;
    //Motor Power


    double drive;
    double turn;

    long pixelTime;
    double strafe;
    double SlidePower;
    double fLeftPow, fRightPow, bLeftPow, bRightPow;

    boolean gameOn = true;


    boolean IntakeReady = false;

    @Override
    public void runOpMode() throws InterruptedException {

        if (opModeInInit()) {
            initCode();
        }
        waitForStart();

        while(opModeIsActive()){

            if (LimitSwitch.isPressed()) {
                telemetry.addData("Intake", "Ready");
                if (!Rumbled) {
                    gamepad2.rumble(500);
                    gamepad2.rumble(500);
                    Rumbled = true;

                }
            } else {
                Rumbled = false;
                gamepad2.stopRumble();
                telemetry.addData("Intake", " Not Ready");
                ;
                ;
            }

            telemetry.addData("ServoR", HangR.getPosition());
            telemetry.addData("ServoL", HangL.getPosition());

            telemetry.addData("SlideL:", SlideL.getCurrentPosition());
            if (Color instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
            }
            telemetry.addData("Pixels:", Pixels);


            //Movement Controller


            right_drivePower = gamepad1.right_stick_y * -1;
            back_left_drivePower = gamepad1.left_stick_y * -1;
            left_drivePower = gamepad1.left_stick_y * -1;
            back_right_drivePower = gamepad1.right_stick_y * -1;


            left_drive.setPower(left_drivePower);
            right_drive.setPower(right_drivePower);
            back_left_drive.setPower(left_drivePower);
            back_right_drive.setPower(right_drivePower);


            boolean rightbumper = gamepad1.right_bumper; //Strafe Right
            boolean leftbumper = gamepad1.left_bumper; //Strafe Left


            boolean UpSlideBumper = gamepad2.right_bumper;
            boolean DownSlideBumper = gamepad2.left_bumper;

            long endTime = System.currentTimeMillis() - startTime;

            //attachments


            if (rightbumper) {

                left_drive.setPower(1); // left drive is 0
                right_drive.setPower(-1); // right drive is 2
                back_left_drive.setPower(-1); // back left drive is 1
                back_right_drive.setPower(1); // back right drive is 3


            } else if (leftbumper) {

                left_drive.setPower(-1);
                right_drive.setPower(1);
                back_left_drive.setPower(1);
                back_right_drive.setPower(-1);


            }
            telemetry.addData("Slides", SlideL.getCurrentPosition());
            telemetry.update();

            //Fwd Bckwd

            //Auto Forward and Backward

       /* if(gamepad1.right_trigger > 0.3){
            left_drive.setPower(1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(1);
        }
        else if(gamepad1.left_trigger > 0.3){
            left_drive.setPower(-1);
            right_drive.setPower(-1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(-1);



        }
        else{

        } */


            //Led code


            if (Pixels == 0 && endTime < 85000) {
                showRed();
            }

            if (Pixels == 1 && endTime < 85000) {
                showGreen();
            }

            if (Pixels == 2 && endTime < 85000) {
                showPurple();
            }

            if (endTime > 80000 && Pixels == 0 || endTime > 80000 && Pixels == 1 || endTime > 80000 && Pixels == 2) {


                if (endTime > 80100 && endTime < 80500) {
                    showGold();
                }
                if (endTime > 80500 && endTime < 81000) {
                    showPurple();
                }
                if (endTime > 81500 && endTime < 82000) {
                    showGold();

                }
                if (endTime > 82000 && endTime < 82500) {
                    showPurple();
                }
                if (endTime > 82500 && endTime < 83000) {
                    showGold();
                }
                if (endTime > 83500 && endTime < 84000) {
                    showPurple();
                }
                if (endTime > 84000 && endTime < 84500) {
                    showGold();

                }
                if (endTime > 85000) {
                    showPurple();
                }


            }

            // CLAW ROTATION


            //Slide Goes Down
            if (!LimitSwitch.isPressed()) {


                if (DownSlideBumper && !yPressed) {


                    IntakeBox();
                    boardAdjust = false;
                    while (!LimitSwitch.isPressed()) {
                        SlideR.setPower(-0.3);
                        SlideL.setPower(-0.3);
                        if(UpSlideBumper){
                            break;
                        }
                    }


                } else if (DownSlideBumper) {
                    SlideR.setPower(-0.1);
                    SlideL.setPower(-0.1);
                }


            } else {
                StopSlides();
                OpenBox();
                IntakeReady = true;


                if (Pixels == 2) {
                    CloseBox();
                }

                //if 0 pixels
                else if (Pixels < 2 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2.25 && (!pixelGap))) {
                    Pixels++;
                    pixelTime = System.currentTimeMillis();
                    pixelGap = true;
                }
                //if one pixel, wait to make sure that were not scanning the same pixel
                else if (Pixels < 2 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2.25 && (pixelGap))) {
                    if (System.currentTimeMillis() > pixelTime + 350) {
                        Pixels++;
                        pixelGap = false;
                    }
                }
            }

            if (UpSlideBumper && !yPressed && !boardAdjust) {
                IntakeReady = false;
                CloseBox();
                IntakeBox();
                SlideR.setPower(0.8);
                SlideL.setPower(0.8);


            } else if (UpSlideBumper) {
                IntakeReady = false;
                SlideR.setPower(0.7);
                SlideL.setPower(0.7);
            } else if (!DownSlideBumper && !LimitSwitch.isPressed()) {

                HoldSlides();


            }


            //Drop Box on Board
            if (gamepad2.y) {
                yPressed = true;
                BoardDropBox();

            }


            // open
            if (gamepad2.left_trigger > 0.3) {
                CloseBox();
            }

            // close
            if (gamepad2.right_trigger > 0.3 && yPressed) {
                OpenBox();
                boardAdjust = true;
                Pixels = 0;
                yPressed = false;

            } else if (gamepad2.right_trigger > 0.3) {
                OpenBox();


            }


            //intake
            if (gamepad2.a && IntakeReady) {
                Intake.setPower(1); // was 0.9 until 3/1
                IntakeRoller.setPower(-0.8);

            }
            // outtake
            else if (gamepad2.b) {
                Intake.setPower(-0.6);
                IntakeRoller.setPower(0.8);
            } else {
                Intake.setPower(0);
                IntakeRoller.setPower(0);
            }

            //drone launching and resetting
            if (gamepad2.back) {
                Drone.setPosition(0.75);
            } else {
                Drone.setPosition(0.55);

            }


            // Hanging / LeadScrew Binds

            if (gamepad2.dpad_up) {
                HangR.setPosition(0.5); // correct
                HangL.setPosition(0.1);

            }
            if (gamepad2.dpad_down) {
                HangR.setPosition(0);
                HangL.setPosition(0.6); //correct

            }
            if(isStopRequested()){
                left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                left_drive.setPower(0);
                right_drive.setPower(0);
                back_left_drive.setPower(0);
                back_right_drive.setPower(0);
            }
        }


            left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            left_drive.setPower(0);
            right_drive.setPower(0);
            back_left_drive.setPower(0);
            back_right_drive.setPower(0);
            colorOff();

    }




    private void CloseBox(){
        BucketHold.setPosition(0); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0.7); //close

    }


    private void useGrabber(){
        Grabber.setPosition(0.7);
    }

    private void setGrabber(){
        Grabber.setPosition(0.35);
    }
    private void BoardDropBox(){
        BucketR.setPosition(0.23);
        BucketL.setPosition(0.27);
    }
    private void HoldSlides(){
        SlideR.setPower(0.35);
        SlideL.setPower(0.35);
    }
    private void StopSlides(){
        SlideR.setPower(0);
        SlideL.setPower(0);
    }
    private void IntakeBox(){
        BucketR.setPosition(0.58);
        BucketL.setPosition(0.62);
//        BucketR.setPosition(0.59);
//        BucketL.setPosition(0.63);
    }

    private void showPurple(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {278 , 89, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGreen(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {101 , 42, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void showGold(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {247 , 98, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    private void showRed(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {6 , 100, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    private void colorOff(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {0 , 0, 0 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    private void initCode(){
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");

        //EXPANSION HUB MOTORS
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");
        Intake = hardwareMap.dcMotor.get("Intake");


        //MOTOR DIRECTION SWITCHING
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);

        //SENSORS
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Color = hardwareMap.get(NormalizedColorSensor.class,"Color");

        //SERVOS
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Drone = hardwareMap.get(Servo.class, "Drone");
        BucketR.setDirection(Servo.Direction.REVERSE);
        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");





        Grabber = hardwareMap.get(Servo.class, "Grab");

        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");

        HangR = hardwareMap.servo.get("HangR");
        HangL = hardwareMap.servo.get("HangL");





        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neopixels");

        neopixels.setNumberOfPixels(NUM_PIXELS);

        //zero Power


        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        HangR.setPosition(0.5); // correct
        HangL.setPosition(0.1);
        // HangR.setPosition(0.5);
        //  HangL.setPosition(0.5);
        //O position for Servos Default
/*
        BucketR.setPosition(0);
        BucketL.setPosition(1);
*/



    }

}