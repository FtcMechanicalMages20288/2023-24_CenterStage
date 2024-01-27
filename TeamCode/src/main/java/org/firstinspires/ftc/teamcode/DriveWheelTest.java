package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "DriveWheelTest", group = "TeleOp")
public class DriveWheelTest extends OpMode {

    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;


    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if(gamepad2.a){
            left_drive.setPower(1);
        }
        if(gamepad2.b){
            right_drive.setPower(1);
        }
        if(gamepad2.y){
            back_left_drive.setPower(1);
        }
        if(gamepad2.x){
            back_right_drive.setPower(1);
        }
    }
}
