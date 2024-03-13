package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Config
@TeleOp(name = "SlideToPos", group = "TeleOp")

public class PIDFarm extends OpMode {

    private final double ticks_in_degrees = 537.71/180;
    public static int target = 0;
    //DriveTrain Motors
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, BucketR, BucketL, Drone;

    private TouchSensor LimitSwitch;

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;



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

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }


    @Override
    public void loop() {

        controller.setPID(p,i,d);
        int slidePos = SlideL.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;

        SlideR.setPower(power);

        telemetry.addData("pos", slidePos);
        telemetry.addData("Target", target);
        telemetry.update();

    }
}
