package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.w8wjb.ftc.AdafruitNeoDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "TeleOp ")
@Disabled

public class RobotClass extends OpMode {

    //DriveTrain Motors
    public  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    public DcMotor SlideR, SlideL, Intake;

    public Servo BucketHold, BucketR, BucketL, Drone, HangR, HangL, Grabber;
    public CRServo IntakeRoller;







    //limitswitch
    public TouchSensor LimitSwitch;
    //BoxColorSensor
    public NormalizedColorSensor Color;

    public   long startTime = System.currentTimeMillis();


    public double left_drivePower;
    public    double right_drivePower;
    public double back_right_drivePower;
    public double back_left_drivePower;

    public int Pixels = 0;

    public static final int NUM_PIXELS = 60;

    public AdafruitNeoDriver neopixels;
    @Override
    public void init() {
        //DRIVE MOTORS
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

    }

    @Override
    public void loop() {
        CloseBox();
    }

    public void CloseBox(){
        BucketHold.setPosition(0); //close
    }
    public void OpenBox(){
        BucketHold.setPosition(0.7); //close

    }


    public void useGrabber(){
        Grabber.setPosition(0.7);
    }

    public void setGrabber(){
        Grabber.setPosition(0.35);
    }
    public void BoardDropBox(){
        BucketR.setPosition(0.23);
        BucketL.setPosition(0.27);
    }
    public void HoldSlides(){
        SlideR.setPower(0.35);
        SlideL.setPower(0.35);
    }
    public void StopSlides(){
        SlideR.setPower(0);
        SlideL.setPower(0);
    }
    public void IntakeBox(){
        BucketR.setPosition(0.58);
        BucketL.setPosition(0.62);
//        BucketR.setPosition(0.59);
//        BucketL.setPosition(0.63);
    }

    public void showPurple(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {278 , 89, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    public void showGreen(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {101 , 42, 66 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    public void showGold(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {247 , 98, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }

    public void showRed(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {6 , 100, 66});
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }
    public void colorOff(){
        int[] colors = new int[NUM_PIXELS];

        for (int i=0; i < colors.length; i++) {
            int color = android.graphics.Color.HSVToColor(new float[] {0 , 0, 0 });
            colors[i] = color;
        }

        neopixels.setPixelColors(colors);
        neopixels.show();
    }


}
