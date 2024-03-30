package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "LeadScrewTest", group = "TeleOp")
public class LeadScrewTest extends OpMode {

    private DcMotor LeadScrew;

    @Override
    public void init() {
        LeadScrew = hardwareMap.dcMotor.get("Lead Screw");
    }


    @Override
    public void loop() {
        if (gamepad2.x) {
            LeadScrew.setPower(0.8);
        }
        if (gamepad2.y) {
            LeadScrew.setPower(-0.8);
        }
        else {
            LeadScrew.setPower(0);
        }
    }
}
