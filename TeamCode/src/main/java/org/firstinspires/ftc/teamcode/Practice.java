package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled // Means that it will not show on the robot controller
@TeleOp(name = "test", group = "TeleOp")

    public class Practice extends OpMode {


    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone, PixelPusher;

    //limitswitch
    private TouchSensor LimitSwitch;
    //BoxColorSensor
    private NormalizedColorSensor Color;

    private ColorSensor c;

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

        right_drive = hardwareMap.dcMotor.get("rm");
        left_drive = hardwareMap.dcMotor.get("lm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");
        Intake = hardwareMap.dcMotor.get("Intake");

        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);

        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");

        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Drone = hardwareMap.get(Servo.class, "Drone");
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Color = hardwareMap.get(NormalizedColorSensor.class,"Color");
        c = hardwareMap.get(ColorSensor.class, "Color2");

        BucketR.setDirection(Servo.Direction.REVERSE);
        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        /*double right_drivePower = gamepad1.left_stick_y;
        double left_drivePower;
        double back_right_drivePower;
        double back_left_drivePower;*/

    }

    public void rest(){
        right_drive.setPower(0);
        back_left_drive.setPower(0);
        left_drive.setPower(0);
        back_right_drive.setPower(0);
    }

    @Override
    public void loop() {
        //if (gamepad1.left_stick_y > 0) {
        right_drive.setPower(gamepad1.right_stick_y);
        left_drive.setPower(gamepad1.left_stick_y);
        back_left_drive.setPower(gamepad1.left_stick_y);
        back_right_drive.setPower(gamepad1.right_stick_y);

            if (gamepad1.a) {
                PixelPusher.setPosition(0.5);
            }
            if (gamepad1.b) {
                PixelPusher.setPosition(0.75);
            }

            if(gamepad1.right_bumper) {
                right_drive.setPower(0.5);
                back_right_drive.setPower(-0.5);
                back_left_drive.setPower(0.5);
                left_drive.setPower(-0.5);
            }
            else if(gamepad1.left_bumper){
                right_drive.setPower(-0.5);
                back_left_drive.setPower(-0.5);
                left_drive.setPower(0.5);
                back_right_drive.setPower(0.5);
            }

        //}
        /*else{
            right_drive.setPower(0);
            left_drive.setPower(0);
            back_left_drive.setPower(0);
            back_right_drive.setPower(0);
        }*/

        telemetry.addData("Distance: ", "%.3f", ((DistanceSensor) Color).getDistance(DistanceUnit.CM));
        telemetry.addData("Color", Color.getNormalizedColors().toColor());

        telemetry.addData("Red", c.red());
        telemetry.addData("Green", c.green());
        telemetry.addData("Blue", c.blue());

        telemetry.update();

      /* if (LimitSwitch.isPressed()){
           gamepad1.rumble(500);
       }
       else{
           gamepad1.stopRumble();
       }
       */

        while (right_drive.isBusy()){
            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());

            telemetry.update();
        }






    }

}