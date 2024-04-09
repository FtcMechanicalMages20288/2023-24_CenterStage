package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "cHubTester", group = "TeleOp")

public class cHubTester extends OpMode {

    private DcMotor motor;
    private Servo servo;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        servo = hardwareMap.get(Servo.class, "servo");

        motor.setPower(0.8);
        servo.setPosition(0.7);
    }

    @Override
    public void loop() {

        // motor
        if (gamepad2.a) {
            motor.setPower(0.8);

        } else {

            motor.setPower(0);
        }


        // servo
        if (gamepad2.b) {
            servo.setPosition(0.7);

        } else {
            servo.setPosition(0);
        }
    }
}
