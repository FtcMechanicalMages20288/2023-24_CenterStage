package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name = "DeadWheelEncoderManual", group = "Robot")
public class DeadWheelEncoderManual extends LinearOpMode {
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;

    static final int tickCount = 2000;
    static final int resetEncoder = 0;

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

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() {
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
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "blm"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lm"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "brm"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        BucketR.setDirection(Servo.Direction.REVERSE);
        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);


        waitForStart();
        if (opModeIsActive()) {
            while(true) {
                telemetry.addData("Encoder Right", rightEncoder.getCurrentPosition());
                telemetry.addData("Encoder Left", leftEncoder.getCurrentPosition());
                telemetry.addData("Encoder Front", frontEncoder.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

