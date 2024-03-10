package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config

@Autonomous(group = "RBR Final")
public class StrafeFix extends LinearOpMode {



    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor  brightDrive = null;
    private DcMotor bleftDrive = null;

    public static double lefty = -0.52;
    public static double blefty = 0.5;

    public static double righty = 0.54;

    public static double brighty = -0.5;

    public static double Rlefty = 0.5;
    public static double Rblefty = -0.5;

    public static double Rrighty = -0.5;

    public static double Rbrighty = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        bleftDrive  = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive  = hardwareMap.get(DcMotor.class, "brm");
        rightDrive = hardwareMap.get(DcMotor.class, "blm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        /*strafeLeft(lefty, blefty, righty, brighty);
        sleep(3000);
        strafeLeft(0, 0, 0,0); */

       strafeRight(Rlefty, Rblefty, Rrighty,Rbrighty);
        sleep(3000);
        strafeRight(0, 0, 0,0);
    }
    public void strafeLeft(double left, double backleft, double right, double backright){
        leftDrive.setPower(left);
        bleftDrive.setPower(backleft);
        rightDrive.setPower(right);
        brightDrive.setPower(backright);
    }

    public void strafeRight(double left, double backleft, double right, double backright){
        leftDrive.setPower(left);
        bleftDrive.setPower(backleft);
        rightDrive.setPower(right);
        brightDrive.setPower(backright);
    }
}
