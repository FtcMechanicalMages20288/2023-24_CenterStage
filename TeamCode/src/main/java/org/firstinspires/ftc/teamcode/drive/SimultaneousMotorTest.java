package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SimulTest", group = "TeleOp")
public class SimultaneousMotorTest extends OpMode {

public DcMotor rm, lm,blm,brm;
    @Override
    public void init() {
        rm = hardwareMap.dcMotor.get("rm");
        lm = hardwareMap.dcMotor.get("lm");
        brm = hardwareMap.dcMotor.get("brm");
        blm = hardwareMap.dcMotor.get("blm");

    }

    @Override
    public void loop() {

           /* for(int i = 1; i < 101; i++ ){
                rm.setPower(i/101);
                lm.setPower(i/101);
                blm.setPower(i/101);
                brm.setPower(i/101);
            }*/
        rm.setPower(1);
        lm.setPower(1);
        blm.setPower(1);
        brm.setPower(1);



    }
}
