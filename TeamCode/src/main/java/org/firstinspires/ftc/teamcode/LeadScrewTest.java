package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "SlideToPos", group = "TeleOp")

public class LeadScrewTest extends OpMode {

    static final double ticks = 537.7;
    //DriveTrain Motors
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;

    private TouchSensor LimitSwitch;



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
      if(gamepad1.a ){
          SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          SlideR.setTargetPosition((int)(ticks/2));
          SlideR.setPower(0.5);
          SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          while (SlideR.isBusy()){
              SlideL.setPower(0.5);
          }

              SlideL.setPower(0.3);
              SlideR.setPower(0.3);

      }
      if(gamepad1.b){
          SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          SlideR.setTargetPosition((int)(ticks/2));
          SlideL.setTargetPosition((int)(ticks/2));
          SlideR.setPower(0.5);
          SlideL.setPower(0.5);
          SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         while (SlideR.isBusy() && SlideL.isBusy()){

          }

          SlideR.setPower(0);
          SlideL.setPower(0);
      }
    }
}
