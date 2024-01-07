package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "MecanumDriveSS24", group = "TeleOp")

public class MecanumDriveSingleStick2024 extends OpMode {


    //DriveTrain Motors
    private  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;

    private DcMotor LeadScrew;
    private CRServo Hook;


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

        LeadScrew = hardwareMap.dcMotor.get("LeadScrew");
        Hook = hardwareMap.get(CRServo.class, "Hook");




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
        if (gamepad1.left_stick_y < 0 || gamepad1.left_stick_y > 0) { //Movement Controller
            right_drivePower = gamepad1.left_stick_y;
            back_left_drivePower = gamepad1.left_stick_y;
            left_drivePower = gamepad1.left_stick_y;
            back_right_drivePower = gamepad1.left_stick_y;
        } else if (gamepad1.right_stick_x == 0) {
            right_drivePower = 0;
            back_left_drivePower = 0;
            left_drivePower = 0;
            back_right_drivePower = 0;
        }

        if (gamepad1.right_stick_x < 0 || gamepad1.right_stick_x > 0) { //Turning Controller
            right_drivePower = -gamepad1.right_stick_x;
            back_left_drivePower = -gamepad1.right_stick_x;
            left_drivePower = gamepad1.right_stick_x;
            back_right_drivePower = gamepad1.right_stick_x;
        } else if (gamepad1.left_stick_y == 0) {
            right_drivePower = 0;
            back_left_drivePower = 0;
            left_drivePower = 0;
            back_right_drivePower = 0;
        }

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
            if (Rumbled = false) {
                gamepad2.rumble(200);
                Rumbled = true;
            }
        } else {
            Rumbled = false;
            telemetry.addData("Intake", " Not Ready");
        }


        //Slide Goes Down
        if (!LimitSwitch.isPressed()) {
            if (DownSlideBumper) {

                IntakeBox();
                SlideR.setPower(-0.3);
                SlideL.setPower(-0.3);

            }
            else if (DownSlideBumper && SlideL.getCurrentPosition() > -120){
                IntakeBox();
                SlideR.setPower(-0.8);
                SlideL.setPower(-0.8);
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
                if (Pixels < 2) {
                    if (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 3) {
                        Pixels++;
                        sleep(1000);
                    }
                }
            }
        }
        if (UpSlideBumper ) {

            Pixels = 0;
            IntakeReady = false;
            CloseBox();
            IntakeBox();
            SlideR.setPower(0.8);
            SlideL.setPower(0.8);


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
            Intake.setPower(0.65);

        }
        // outtake
        else if(gamepad2.b){
            Intake.setPower(-0.8);

        }

        else Intake.setPower(0);

        //drone launching and resetting
        if (gamepad2.x) {
            Drone.setPosition(0.8);
        }
        else{
            Drone.setPosition(0.4);
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
        BucketHold.setPosition(0.7); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0); //close
    }
    private void BoardDropBox(){
        BucketR.setPosition(0.85);
        BucketL.setPosition(0.85);
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