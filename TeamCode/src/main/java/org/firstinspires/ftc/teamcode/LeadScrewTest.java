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

@TeleOp(name = "LeadScrewTest", group = "TeleOp")

public class LeadScrewTest extends OpMode {


    //DriveTrain Motors
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
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
        Color = hardwareMap.get(NormalizedColorSensor.class, "Color");
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
    }
}
