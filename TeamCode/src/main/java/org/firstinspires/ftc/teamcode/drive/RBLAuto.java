package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(group = "R BL Auto")
public class RBLAuto extends LinearOpMode {

    private Servo BucketHold, BucketR, BucketL;
    private DcMotor SlideR, SlideL, Intake;

   public static double slidePower = 0.45;
    int xValue = 19;
   int yValue = -10;

    public static int x2Value = 26;
    public static int y2Value = 4 ;

    public static int bw = 10;

    public static int turn2 = 110;

   public static int waitTime = 750;
    public static int waitTimev2 = 750;

   public static double FwBw = 8;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();



        Pose2d sP = new Pose2d(0,0,0);

        drive.setPoseEstimate(sP);








        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)
                .lineTo(new Vector2d(xValue, yValue))
                .back(7)
                .lineTo(new Vector2d(20, -30))
                .turn(Math.toRadians(110))
                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(pos1.end())
                .forward(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();





        TrajectorySequence pos1p3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .strafeRight(15)


                .build();



        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                .lineTo(new Vector2d(x2Value, y2Value))
                .back(bw)
                .lineTo(new Vector2d(28.4, -30))
                .turn(Math.toRadians(turn2))
                .build();



        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .strafeTo(new Vector2d(45,-26))


                .build();












        waitForStart();

        if(!isStopRequested()){
           /* drive.followTrajectorySequence(pos1);

            IntakeBox();
            SlidePower(slidePower);
            sleep(waitTime);
            HoldSlides();
            BoardDropBox();
            sleep(waitTimev2);
            drive.followTrajectorySequence(traj2);
            OpenBox();
            sleep(waitTimev2);
            drive.followTrajectorySequence(traj3);

            */

            drive.followTrajectorySequence(pos2);

            IntakeBox();
            SlidePower(slidePower);
            sleep(waitTime);
            HoldSlides();
            BoardDropBox();
            sleep(waitTimev2);
            drive.followTrajectorySequence(traj2);
            OpenBox();
            sleep(waitTimev2);
            drive.followTrajectorySequence(traj3);

        }




    }
    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");




        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketR.setDirection(Servo.Direction.REVERSE);
    }
    private void SlidePower(double p){
        SlideR.setPower(p);
        SlideL.setPower(p);





    }
    private void CloseBox(){
        BucketHold.setPosition(0.7); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0); //close
    }
    private void BoardDropBox(){
        BucketR.setPosition(0.85);
        BucketL.setPosition(0.85);
    }
    private void HoldSlides(){
        SlideR.setPower(0.1);
        SlideL.setPower(0.1);
    }
    private void StopSlides(){
        SlideR.setPower(0);
        SlideL.setPower(0);
    }
    private void IntakeBox(){
        BucketR.setPosition(0.5);
        BucketL.setPosition(0.5);
    }


}

