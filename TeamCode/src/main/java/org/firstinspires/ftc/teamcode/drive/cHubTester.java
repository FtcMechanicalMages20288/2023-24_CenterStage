package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "cHubTester", group = "TeleOp")

public class cHubTester extends OpMode {

    private DcMotor motor;
    private Servo BucketHold;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
       motor = hardwareMap.dcMotor.get("rm");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");




    }
}
