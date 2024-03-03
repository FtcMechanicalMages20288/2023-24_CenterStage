package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Auto")
public class PixelStacker extends LinearOpMode {
    public static double DISTANCE = 8; // in
    private Servo BucketHold, BucketR, BucketL, Drone, HangR, HangL, Grabber;

    private DcMotor Intake;
    private CRServo IntakeRoller;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence grabp1 = drive.trajectorySequenceBuilder(new Pose2d())
                .addDisplacementMarker(0, () -> {
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                    IntakeBox();
                })
                .back(DISTANCE)
                .addDisplacementMarker(8, () -> {
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                    useGrabber();

                })



                .forward(DISTANCE+3,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .addDisplacementMarker(19, () -> {
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                    setGrabber();
                    Intake.setPower(1);
                    IntakeRoller.setPower(-0.8);

                })

                .back(DISTANCE + 2,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .addDisplacementMarker(29, () -> {
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!

                    Intake.setPower(0);
                    IntakeRoller.setPower(0);

                })




                .lineTo((new Vector2d(5,-10)))




                .build();

        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        Grabber = hardwareMap.get(Servo.class, "Grab");
        Intake = hardwareMap.dcMotor.get("Intake");
        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");

        waitForStart();

        if (isStopRequested()) return;
        setGrabber();
        IntakeBox();
        OpenBox();


        drive.followTrajectorySequence(grabp1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    private void IntakeBox(){
        BucketR.setPosition(0.61);
        BucketL.setPosition(0.65);

    }
    private void OpenBox(){
        BucketHold.setPosition(0.7); //close

    }

    private void useGrabber(){
        Grabber.setPosition(0.9);
    }

    private void setGrabber(){
        Grabber.setPosition(0.35);
    }
}
