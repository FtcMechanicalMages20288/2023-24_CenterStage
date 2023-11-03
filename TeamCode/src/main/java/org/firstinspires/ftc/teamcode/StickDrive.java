package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StickDrive", group = "TeleOp")
public class StickDrive extends OpMode {
    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;


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

    }

    @Override
    public void loop() {
        right_drivePower = gamepad1.left_stick_y *-1 ;
        back_left_drivePower = gamepad1.left_stick_y;
        left_drivePower = gamepad1.left_stick_y;
        back_right_drivePower = gamepad1.left_stick_y * -1;

    }
}