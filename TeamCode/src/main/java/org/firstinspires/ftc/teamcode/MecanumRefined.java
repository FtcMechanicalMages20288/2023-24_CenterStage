package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Mecanum Refined", group = "TeleOp")

public class MecanumRefined extends OpMode {


    //DriveTrain Motors
    private  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone, HangR, HangL;

    private DcMotor LeadScrew;
    private CRServo Hook;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    //limitswitch
    private TouchSensor LimitSwitch;
    //BoxColorSensor
    private NormalizedColorSensor Color;

    long startTime = System.currentTimeMillis();




    int Pixels = 0;


    boolean Rumbled = false;

    boolean yPressed = false;

    boolean boardAdjust = false;
    //Motor Power


    double drive;
    double turn;
    double strafe;
    double SlidePower;
    double fLeftPow, fRightPow, bLeftPow, bRightPow;




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
        LeadScrew = hardwareMap.dcMotor.get("LeadScrew");

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
        Hook = hardwareMap.get(CRServo.class, "Hook");


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        HangR.setPosition(0.5);
        HangL.setPosition(0.5);
        //O position for Servos Default
/*
        BucketR.setPosition(0);
        BucketL.setPosition(1);
*/


    }


    @Override
    public void loop() {

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
            telemetry.addData("Intake", " Not Ready");;;
        }

        telemetry.addData("ServoR", HangR.getPosition());
        telemetry.addData("ServoL", HangL.getPosition());

        telemetry.addData("SlideL:", SlideL.getCurrentPosition());
        if (Color instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
        }
        telemetry.addData("Pixels:", Pixels);



        //Movement Controller


        drive = gamepad1.left_stick_y * -1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        fLeftPow = Range.clip(drive + turn + strafe, -1, 1);
        bLeftPow = Range.clip(drive + turn - strafe, -1, 1);
        fRightPow = Range.clip(drive - turn - strafe, -1, 1);
        bRightPow = Range.clip(drive - turn + strafe, -1, 1);

        left_drive.setPower(fLeftPow);
        back_left_drive.setPower(bLeftPow);
        right_drive.setPower(fRightPow);
        back_right_drive.setPower(bRightPow);


        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left




        boolean UpSlideBumper = gamepad2.right_bumper;
        boolean DownSlideBumper = gamepad2.left_bumper;

        long endTime = System.currentTimeMillis() - startTime;

        //attachments


        if (rightbumper) {

            left_drive.setPower(-1); // left drive is 0
            right_drive.setPower(1); // right drive is 2
            back_left_drive.setPower(1); // back left drive is 1
            back_right_drive.setPower(-1); // back right drive is 3




        } else if (leftbumper) {

            left_drive.setPower(1);
            right_drive.setPower(-1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(1);




        }
        telemetry.addData("Slides", SlideL.getCurrentPosition());
        telemetry.update();

        //Led code


        if(Pixels == 0 && endTime < 85000){
            pattern =  RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
        }

        if(Pixels == 1 && endTime < 85000){
            pattern =  RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(pattern);
        }

        if(Pixels == 2 && endTime < 85000){
            pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            blinkinLedDriver.setPattern(pattern);
        }



        if(endTime > 80000 && Pixels == 0 || endTime > 80000 && Pixels == 1 || endTime > 80000 && Pixels == 2){



            if(endTime > 90000 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }

            if(endTime > 80100 && endTime < 80500 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(pattern);
            }
            if(endTime >80500  && endTime <  81000 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }
            if(endTime >81500  && endTime <82000  ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(pattern);

            }
            if(endTime > 82000  && endTime < 82500  ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }
            if(endTime > 82500  && endTime < 83000 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(pattern);
            }
            if(endTime >83500  && endTime <84000  ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }
            if(endTime >84000  && endTime < 84500 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(pattern);

            }
            if(endTime > 85000 ){
                pattern =  RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(pattern);
            }



        }

        // CLAW ROTATION




        //Slide Goes Down
        if (!LimitSwitch.isPressed()) {


            if (DownSlideBumper && !yPressed) {


                IntakeBox();
                while (!LimitSwitch.isPressed()){
                    SlideR.setPower(-0.3);
                    SlideL.setPower(-0.3);
            }
                boardAdjust = false;

            }
            else if(DownSlideBumper){
                SlideR.setPower(-0.1);
                SlideL.setPower(-0.1);
            }


        } else {
            StopSlides();
            OpenBox();


            if (Pixels == 2) {
                CloseBox();
            }


            if (Pixels < 2 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2.25)) {
                    Pixels++;
                    sleep(350);
            }
        }

        if (UpSlideBumper && !yPressed && !boardAdjust ) {

            CloseBox();
            IntakeBox();
            SlideR.setPower(0.8);
            SlideL.setPower(0.8);


        }
        else if (UpSlideBumper){
            SlideR.setPower(0.7);
            SlideL.setPower(0.7);
        }

        else if (!DownSlideBumper && !LimitSwitch.isPressed()){

            HoldSlides();


        }




        //Drop Box on Board
        if(gamepad2.y && Pixels == 2 || gamepad2.y && Pixels == 1){
            yPressed = true;
            BoardDropBox();

        }



        // open
        if(gamepad2.left_trigger > 0.3) {
            CloseBox();
        }

        // close
        if(gamepad2.right_trigger > 0.3 && yPressed) {
            OpenBox();
            boardAdjust = true;
            Pixels = 0;
            yPressed = false;

        }
        else if(gamepad2.right_trigger > 0.3){
            OpenBox();
        }


        //Auto Forward and Backward

        if(gamepad1.right_trigger > 0.3){
            left_drive.setPower(-1);
            right_drive.setPower(-1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(-1);
        }
        if(gamepad1.left_trigger > 0.3){

            left_drive.setPower(1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(1);

        }

        //intake
        if(gamepad2.a) {
            Intake.setPower(.9);

        }
        // outtake
        else if(gamepad2.b){
            Intake.setPower(-0.6);

        }

        else Intake.setPower(0);

        //drone launching and resetting
        if (gamepad2.back) {
            Drone.setPosition(0.3);
        }
        else{
            Drone.setPosition(0.55);

        }




        // Hanging / LeadScrew Binds
        if(gamepad2.dpad_right) {
            LeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LeadScrew.setPower(1);

        }
        else if (gamepad2.dpad_left) {
            LeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LeadScrew.setPower(-1);

        }
        else{
            LeadScrew.setPower(0);
            LeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(gamepad2.dpad_down){
            Hook.setPower(0.5);
        }
        else if(gamepad2.dpad_up){
            Hook.setPower(-0.5);

        }
        else{
            Hook.setPower(0);
        }






    }


    private void CloseBox(){
        BucketHold.setPosition(0); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0.7); //close

    }
    private void BoardDropBox(){
        BucketR.setPosition(0.15);
        BucketL.setPosition(0.15);
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
        BucketR.setPosition(0.5);
        BucketL.setPosition(0.5);
    }
}