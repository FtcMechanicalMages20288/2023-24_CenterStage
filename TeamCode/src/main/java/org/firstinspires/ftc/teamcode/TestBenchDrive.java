package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TestBenchDrive", group = "TeleOp")

public class TestBenchDrive  extends OpMode {




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

            right_drive.setDirection(DcMotor.Direction.REVERSE);
            back_right_drive.setDirection(DcMotor.Direction.REVERSE);


            //O position for Servos Default
/*
        BucketR.setPosition(0);
        BucketL.setPosition(1);
*/


        }


        @Override
        public void loop() {
            //Movement Controller
            right_drivePower = gamepad1.left_stick_y;
            back_left_drivePower = gamepad1.right_stick_y;
            left_drivePower = gamepad1.right_stick_y;
            back_right_drivePower = gamepad1.left_stick_y;


            left_drive.setPower(left_drivePower*-1);
            right_drive.setPower(right_drivePower*-1);
            back_left_drive.setPower(left_drivePower*-1);
            back_right_drive.setPower(right_drivePower*-1);


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
        }
    }

