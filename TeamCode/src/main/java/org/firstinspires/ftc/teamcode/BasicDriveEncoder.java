package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "Basic Drive Encoder", group = "Auto")
public class BasicDriveEncoder extends LinearOpMode {
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;

    static final int tickCount = 538;


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
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Color = hardwareMap.get(NormalizedColorSensor.class, "Color");
        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Intake = hardwareMap.dcMotor.get("Intake");
        BucketR.setDirection(Servo.Direction.REVERSE);

        SlideL.setMode(DcMotor.RunMode.RESET_ENCODERS);







        waitForStart();


        right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double circumference = 3.14 * 3.78;
        double rotationsNeeded;
        rotationsNeeded = 20/ circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded * 538);

        right_drive.setTargetPosition(-encoderDrivingTarget);
        left_drive.setTargetPosition(-encoderDrivingTarget);
        back_left_drive.setTargetPosition(-encoderDrivingTarget);
        back_right_drive.setTargetPosition(-encoderDrivingTarget);


        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right_drive.setPower(0.2);
        left_drive.setPower(0.2);
        back_left_drive.setPower(0.2);
        back_right_drive.setPower(0.2);

        while(right_drive.isBusy() || left_drive.isBusy() || back_left_drive.isBusy() || back_right_drive.isBusy()){
            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.addData("Target:", encoderDrivingTarget);
            telemetry.update();
        }

        right_drive.setPower(0);
        left_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        //driveEncoder(31);
        //   turnEncoder(10,0.3);

    }
    //to go backward reverse the distance and make the power negative
    //distance in inches
    //power is scaled from -1 to 1
    public void driveEncoder(double distance){
        right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);



// manual set target position


        double circumference = 3.14 * 3.78;
        double rotationsNeeded;
        rotationsNeeded = -(distance / circumference);
        int encoderDrivingTarget = (int) (rotationsNeeded * 538);



        right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        left_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        back_left_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        back_right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));

        int right_front_pos = right_drive.getCurrentPosition();

        right_drive.setPower(0.2);
        left_drive.setPower(0.2);
        back_left_drive.setPower(0.2);
        back_right_drive.setPower(0.2);

        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        while(right_front_pos > encoderDrivingTarget) {
            right_drive.setPower(0.2);
            left_drive.setPower(0.2);
            back_left_drive.setPower(0.2);
            back_right_drive.setPower(0.2);

            right_front_pos = right_drive.getCurrentPosition();
            left_drive.getCurrentPosition();
            back_left_drive.getCurrentPosition();
            back_right_drive.getCurrentPosition();



            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.addData("encoderDrivingTarget: ", encoderDrivingTarget);
            telemetry.update();




            //turnEncoder(10,0.3);

        }


        right_drive.setPower(0);
        left_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        telemetry.addData("Trajectory","finished");
    }

    private void rightTurn(){

    }
    private void leftTurn(){

    }

    private void turnEncoder(double distance, double power){
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.78;
        double LrotationsNeeded = -distance/circumference;
        int LencoderDrivingTarget = (int)(LrotationsNeeded*tickCount);

        double RrotationsNeeded = distance/circumference;
        int RencoderDrivingTarget = (int)(RrotationsNeeded*tickCount);

        right_drive.setTargetPosition(-RencoderDrivingTarget);
        left_drive.setTargetPosition(LencoderDrivingTarget);
        back_left_drive.setTargetPosition(LencoderDrivingTarget);
        back_right_drive.setTargetPosition(-RencoderDrivingTarget);

        right_drive.setPower(-power);
        left_drive.setPower(power);
        back_left_drive.setPower(power);
        back_right_drive.setPower(-power);

        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(right_drive.isBusy() || left_drive.isBusy() || back_left_drive.isBusy() || back_right_drive.isBusy()){
            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.update();
        }

        right_drive.setPower(0);
        left_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        telemetry.addData("Trajectory","finished");
    }
}

