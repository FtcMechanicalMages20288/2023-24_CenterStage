package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(group = "TeleOp ")
public class testRobotClass extends OpMode {

     RobotClass r = new RobotClass();
    @Override
    public void init() {

    }

    @Override
    public void loop() {


        r.BoardDropBox();
        r.CloseBox();
        r.HoldSlides();
    }
}
