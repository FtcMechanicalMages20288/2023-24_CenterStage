package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HubTest", group = "TeleOp")

public class HubTest extends OpMode {
    private DcMotor m0, m1, m2, m3, em0, em1, em2, em3;
    private CRServo s0, s1, s2, s3, s4, s5, es0, es1, es2, es3, es4, es5;


    @Override
    public void init() {

        //initialize motors

        m0 = hardwareMap.dcMotor.get("m0");
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");




/*
        em0 = hardwareMap.dcMotor.get("em0");
        em1 = hardwareMap.dcMotor.get("em1");
        em2 = hardwareMap.dcMotor.get("em2");
        em3 = hardwareMap.dcMotor.get("em3");


 */


        //initialize servos

        s0 = hardwareMap.crservo.get("s0");
        s1 = hardwareMap.crservo.get("s1");
        s2 = hardwareMap.crservo.get("s2");
        s3 = hardwareMap.crservo.get("s3");
        s4 = hardwareMap.crservo.get("s4");
        s5 = hardwareMap.crservo.get("s5");




/*
        es0 = hardwareMap.crservo.get("es0");
        es1 = hardwareMap.crservo.get("es1");
        es2 = hardwareMap.crservo.get("es2");
        es3 = hardwareMap.crservo.get("es3");
        es4 = hardwareMap.crservo.get("es4");
        es5 = hardwareMap.crservo.get("es5");


 */



    }

    @Override
    public void loop() {

        //move all motors

        m0.setPower(0.5);
        m1.setPower(0.5);
        m2.setPower(0.5);
        m3.setPower(0.5);




/*
        em0.setPower(0.5);
        em1.setPower(0.5);
        em2.setPower(0.5);
        em3.setPower(0.5);

 */



        //move all servos


/*
        es0.setPower(0.5);
        es1.setPower(0.5);
        es2.setPower(0.5);
        es3.setPower(0.5);
        es4.setPower(0.5);
        es5.setPower(0.5);

 */

        s0.setPower(0.5);
        s1.setPower(0.5);
        s2.setPower(0.5);
        s3.setPower(0.5);
        s4.setPower(0.5);
        s5.setPower(0.5);



    }
}
