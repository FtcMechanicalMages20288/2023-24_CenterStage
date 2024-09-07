package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled // Means that it will not show on the robot controller
@TeleOp(name = "MecanumDriveLearninglol", group = "TeleOp")

public class MecanumDriveLearning extends OpMode {


    //DriveTrain Motors
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;


    //limitswitch
    private TouchSensor LimitSwitch;
    //BoxColorSensor
    private NormalizedColorSensor Color;

    int Pixels = 0;

    boolean IntakeDelay = true;
    boolean AutoHold = true;
    boolean Rumbled = false;
    boolean frontDrive = true;
    boolean IntakeReady = false;

    //Motor Power
    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;


    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");

        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);

        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Color = hardwareMap.get(NormalizedColorSensor.class,"Color");
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Intake = hardwareMap.dcMotor.get("Intake");
        BucketR.setDirection(Servo.Direction.REVERSE);
        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);




        //O position for Servos Default
/*
        BucketR.setPosition(0);
        BucketL.setPosition(1);
*/


    }


    @Override
    public void loop() {


            telemetry.addData("SlideL:", SlideL.getCurrentPosition());
        if (Color instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
        }
            telemetry.addData("Pixels:", Pixels);

            //Movement Controller
            right_drivePower = gamepad1.right_stick_y;
            back_left_drivePower = gamepad1.left_stick_y;
            left_drivePower = gamepad1.left_stick_y;
            back_right_drivePower = gamepad1.right_stick_y;


            left_drive.setPower(left_drivePower);
            right_drive.setPower(right_drivePower);
            back_left_drive.setPower(left_drivePower);
            back_right_drive.setPower(right_drivePower);


            boolean rightbumper = gamepad1.right_bumper; //Strafe Right
            boolean leftbumper = gamepad1.left_bumper; //Strafe Left



            boolean UpSlideBumper = gamepad2.right_bumper;
            boolean DownSlideBumper = gamepad2.left_bumper;


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


            // CLAW ROTATION


            if (LimitSwitch.isPressed()) {
                telemetry.addData("Intake", "Ready");
                if(Rumbled = false){
                    gamepad2.rumble(200);
                    Rumbled= true;
                }
            } else {
                Rumbled = false;
                telemetry.addData("Intake", " Not Ready");
            }





            //Slide Goes Down
            if(!LimitSwitch.isPressed()){
                if(DownSlideBumper && (SlideL.getCurrentPosition() > -49)){
                    if(IntakeDelay = true){
                        BucketR.setPosition(0.395);
                        BucketL.setPosition(0.33);
                        SlideR.setPower(-0.1);
                        SlideL.setPower(-0.1);
                        sleep(750);
                        SlideR.setPower(-0.6);
                        SlideL.setPower(-0.6);
                        IntakeDelay = false;
                    }
                    else {

                        SlideR.setPower(-0.6);
                        SlideL.setPower(-0.6);
                    }

                } else if (DownSlideBumper && (SlideL.getCurrentPosition() < -49)){


                    BucketR.setPosition(0.515);
                    BucketL.setPosition(0.45);
                    SlideR.setPower(-0.4);
                    SlideL.setPower(-0.4);
                }

            } else {
                StopSlides();
                IntakeReady = true;
                OpenBox();
                AutoHold = true;
                if (IntakeReady = true) {
                    if (Pixels == 2) {
                        CloseBox();
                    }
                    if (Pixels < 2 ) {
                        if (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 3) {
                            Pixels++;
                            sleep(1000);
                        }
                    }
                }
            }
            if (UpSlideBumper && (SlideL.getCurrentPosition() > -49)) {

                BucketR.setPosition(0.395);
                BucketL.setPosition(0.33);
                SlideR.setPower(0.8);
                SlideL.setPower(0.8);

                AutoHold = true;
                Pixels = 0;


            } else if (UpSlideBumper && (SlideL.getCurrentPosition() < -49)) {
                IntakeDelay = true;
                IntakeReady = false;
                if (AutoHold = true) {
                    BucketR.setPosition(0.515);
                    BucketL.setPosition(0.45);
                    SlideR.setPower(0.8);
                    SlideL.setPower(0.8);
                    CloseBox();
                    AutoHold = false;
                } else{
                    BucketR.setPosition(0.515);
                    BucketL.setPosition(0.45);
                    SlideR.setPower(0.8);
                    SlideL.setPower(0.8);

                }

            } else if (!DownSlideBumper && !LimitSwitch.isPressed()){

                HoldSlides();


            }




            //Drop Box on Board
            if(gamepad2.y){
                BoardDropBox();

            }



            // open
            if(gamepad2.left_trigger > 0.3) {
                CloseBox();
            }

            // close
            if(gamepad2.right_trigger > 0.3) {
                OpenBox();
            }

            //intake
            if(gamepad2.a) {
                Intake.setPower(0.8);

            }
            // outtake
            else if(gamepad2.b){
                Intake.setPower(-0.8);

            }

            else Intake.setPower(0);

            if (gamepad2.dpad_down) {
                Drone.setPosition(1);
            }
            else{
                Drone.setPosition(0.7);
            }





    }


    private void CloseBox(){
        BucketHold.setPosition(0.1); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0.3); //close
    }
    private void BoardDropBox(){
        BucketR.setPosition(0.115);
        BucketL.setPosition(0.05);
    }
    private void HoldSlides(){
        SlideR.setPower(0.35);
        SlideL.setPower(0.35);
    }
    private void StopSlides(){
        SlideR.setPower(0);
        SlideL.setPower(0);
    }
}