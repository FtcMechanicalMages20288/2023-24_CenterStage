package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.objdetect.Board;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(group = "RBR_World")
public class RBR_World extends LinearOpMode {

    int dropPos1;
    int dropPos2;
    int dropPos3;
    int finalDropPos;


    // EDIT THIS VARIABLE DEPENDING ON THE OTHER TEAMS' NEEDS
    // THIS CHANGES WHEN THE PIXEL GRABBER IS USED
    double timeAdjust = 0;

    // private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    // private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    // private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    // boolean targetFound = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "WorldyBlue.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)


    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    int DESIRED_TAG_ID = 3;   // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound = false;
    boolean pixelCheck = false;


    boolean aprilAdjust = true;


    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor brightDrive = null;
    private DcMotor bleftDrive = null;

    private ElapsedTime runtime = new ElapsedTime();


    private VisionPortal visionPortal1;
    private CRServo GrabRoller;


    private TouchSensor LimitSwitch;

    private Servo Grabber;

    private CRServo IntakeRoller;

    private NormalizedColorSensor RampSensor, Color, ColorFront;


    // Tfod Stuff Above

    private Servo BucketHold, BucketR, BucketL, PixelPusher;
    private DcMotor SlideR, SlideL, Intake;

    public static double slidePower = 0.45;
    int xValue = 19;
    int yValue = -10;

    public static int x2Value = 26;
    public static int y2Value = 4;

    public static int pos2startx = 30;
    public static int pos2starty = 2;

    public static int strafecorrect = 8;

    public static int fwdDistancep1 = 30;
    public static int fwdDistancep2 = 50;

    public static int cycleX = 30;
    public static int cycleY = 30;
    public static int elapsedTimevar = 5000;
    int pixels = 1;


    public static double DISTANCE = 8; // in

    public static int waitTime = 700;
    public static int waitTimev2 = 750;
    public static int waitTimev3 = 1000;
    public static int WaitageTimeVar = 0;

    public static double FwBw = 8;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();
        initDoubleVision();
        sleep(3000);
        PixelPusher.setPosition(0.45);
        Grabber.setPosition(0.15);
        IntakeBox();
        OpenBox();


        //BoardDropBox();






        // 5

        Pose2d sP = new Pose2d(0, 0, 0);
        Pose2d eel = drive.getPoseEstimate();
        drive.setPoseEstimate(sP);


        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(sP)


                .lineToLinearHeading(new Pose2d(23, -13.3))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                .back(9)
                .strafeLeft(16.3)
                .lineToLinearHeading(new Pose2d(48, 0.5))
                .turn(Math.toRadians(84))
                .lineToLinearHeading(new Pose2d(48.5, -14, Math.toRadians(90))) //Change
                .back(3.7)


                .build();

        TrajectorySequence pos3_intake = drive.trajectorySequenceBuilder(pos3.end())

                .back(1.3) //Change



                .addDisplacementMarker(() -> {
                    Intake.setPower(0.8);
                    IntakeRoller.setPower(-0.8);
                    stopRobot();
                    sleep(500);
                })

                .addDisplacementMarker(() -> {

                    boolean switcheroo = true;
                    long timeOut = System.currentTimeMillis();

                    while (switcheroo && (timeOut + 2000) > (System.currentTimeMillis())) { // Loop while switcheroo is true and 5 seconds have not passed

                        if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) < 4 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2))) {
                           /* Intake.setPower(-1);
                            IntakeRoller.setPower(0.8); */
                            pixels++;
                            switcheroo = false;


                        }
                        else if((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                                (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)){
                            pixels++;
                            switcheroo = false;

                        }
                    }
                    if(!switcheroo){
                        Intake.setPower(-1);
                    }


                })

                .waitSeconds(0.5)

                .build();



        TrajectorySequence pos3_deposit = drive.trajectorySequenceBuilder(pos3_intake.end())

                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    Intake.setPower(-0.8);
                    IntakeRoller.setPower(0.8);

                })

                .lineToConstantHeading(new Vector2d(51, 72))

                .strafeLeft(27)
                .forward(10)

                .build();





        TrajectorySequence pos3_depositp2 = drive.trajectorySequenceBuilder(pos3_deposit.end())
                .forward(8.5,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();

        TrajectorySequence pos3_depositp3 = drive.trajectorySequenceBuilder(pos3_depositp2.end())
                .back(4)
                .strafeRight(11)//10 bf
                .forward(3.3,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )






                .build();

        TrajectorySequence pos3_traj2 = drive.trajectorySequenceBuilder(pos3_depositp3.end())

                .waitSeconds(0.5)
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                /*
                .addDisplacementMarker(() -> {

                    OpenBox();
                    sleep(200);
                    SlidePower(slidePower + 0.2);
                    sleep(400);//Change
                    HoldSlides();
                    IntakeBox();

                })

                .back(0.1,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                 */


                .build();

        TrajectorySequence pos3_park = drive.trajectorySequenceBuilder(pos3_traj2.end())

                //.strafeRight(13)
                //.forward(15)
                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
                .lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf

                .build();



        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)


                .lineToLinearHeading(new Pose2d(20, 0))
                .turn(Math.toRadians(50 - 11))
                .forward(8)
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                .back(8)

                //.lineTo(new Vector2d(18, 2))
                .lineToLinearHeading(new Pose2d(49, -2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(49, -14, Math.toRadians(87)))
                .back(4.5)

                .build();


        TrajectorySequence pos1_intake = drive.trajectorySequenceBuilder(pos1.end())

                .back(0.3)//3.3

                .addDisplacementMarker(() -> {
                    Intake.setPower(0.8);
                    IntakeRoller.setPower(-0.8);
                    stopRobot();
                    sleep(500);
                })

                .addDisplacementMarker(() -> {

                    boolean switcheroo = true;
                    long timeOut = System.currentTimeMillis();

                    while (switcheroo && (timeOut + 2000) > (System.currentTimeMillis())) { // Loop while switcheroo is true and 5 seconds have not passed

                        if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) < 4 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2))) {
                           /* Intake.setPower(-1);
                            IntakeRoller.setPower(0.8); */
                            pixels++;
                            switcheroo = false;


                        }
                        else if((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                                (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)){
                            pixels++;
                            switcheroo = false;

                        }
                    }
                    if(!switcheroo){
                        Intake.setPower(-1);
                    }


                })

                .waitSeconds(0.5)



                .build();

        TrajectorySequence pos1_deposit = drive.trajectorySequenceBuilder(pos1_intake.end())

                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    Intake.setPower(-0.8);
                    IntakeRoller.setPower(0.8);

                })

                .lineToConstantHeading(new Vector2d(51, 72))

                .strafeLeft(15)
                .forward(10)


                //change


                .build();

        //change


        TrajectorySequence pos1_depositp2 = drive.trajectorySequenceBuilder(pos1_deposit.end())
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .strafeLeft(14)


                .forward(14,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .build();


        TrajectorySequence pos1_traj2 = drive.trajectorySequenceBuilder(pos1_depositp2.end())
                .waitSeconds(0.5)
                .back(0.2,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .back(0.1,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )




                .build();

        TrajectorySequence pos1_cycle = drive.trajectorySequenceBuilder(pos1_deposit.end())

                .lineToConstantHeading(new Vector2d(51, 75))
                .lineToLinearHeading(new Pose2d(47, -2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90)))
                .back(7)



                .build();


        TrajectorySequence pos1_cycle_adjust = drive.trajectorySequenceBuilder(pos1_deposit.end())
                .addDisplacementMarker(() -> {
                    SlideDown();
                })
                .lineToConstantHeading(new Vector2d(51, 75))
                .strafeRight(4)
                .build();

        TrajectorySequence pos1_park = drive.trajectorySequenceBuilder(pos1_traj2.end())

                //.strafeRight(25)
                //.forward(15)
                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
                .lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf

                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(pos2startx, pos2starty))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                /*.back(4)
                .turn(Math.toRadians(88))*/
                .lineToLinearHeading(new Pose2d(25, -15.5, Math.toRadians(88)))

                //.lineToLinearHeading(new Pose2d(26, 32, Math.toRadians(88)))
                //.waitSeconds(4) // Change depending on teammate speed

                .build();



        TrajectorySequence pos2_intake = drive.trajectorySequenceBuilder(pos2.end())

                .back(4) //prev 4.2



                .addDisplacementMarker(() -> {
                    Intake.setPower(0.8);
                    IntakeRoller.setPower(-0.8);
                    stopRobot();
                    sleep(500);
                })

                .addDisplacementMarker(() -> {

                    boolean switcheroo = true;
                    long timeOut = System.currentTimeMillis();

                    while (switcheroo && (timeOut + 2000) > (System.currentTimeMillis())) { // Loop while switcheroo is true and 5 seconds have not passed

                        if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) < 4 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2))) {
                           /* Intake.setPower(-1);
                            IntakeRoller.setPower(0.8); */
                            pixels++;
                            switcheroo = false;


                        }
                        else if((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                                (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)){
                            pixels++;
                            switcheroo = false;

                        }
                    }
                    if(!switcheroo){
                        Intake.setPower(-1);
                    }


                })


                .waitSeconds(0.5)

                .build();


        TrajectorySequence pos2_deposit = drive.trajectorySequenceBuilder(pos2_intake.end())


                .lineToLinearHeading(new Pose2d(26, 75, Math.toRadians(90)))

//                .addDisplacementMarker(() -> {
//                    IntakeRoller.setPower(0.8);
//                    Intake.setPower(-1);
//                    sleep(2000);
//
//                })
                .waitSeconds(0.2)
                .addDisplacementMarker(() ->{
                    CloseBox();
                    IntakeBox();
                    SlidePower(slidePower);
                    sleep(waitTime + 250);
                    HoldSlides();
                    BoardDropBox();
                })

                .waitSeconds(.95)
                //.lineToLinearHeading(new Pose2d(30, 80, Math.toRadians(88)))

                .lineToLinearHeading(new Pose2d(32.5, 77, Math.toRadians(90)))

                /*
                .waitSeconds(0.3)
                .strafeLeft(5)


                 */



                .forward(10,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                //.back(3)
                .strafeRight(2.5)
                .waitSeconds(0.5)
                .forward(3)
                .waitSeconds(0.2)



                .build();


        TrajectorySequence pos2_depositp3 = drive.trajectorySequenceBuilder(pos2_deposit.end())
                .back(4,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .strafeLeft(8)//Has been Changed
                .forward(4,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .waitSeconds(0.5)

                .build();



        TrajectorySequence pos2_traj2 = drive.trajectorySequenceBuilder(pos2_deposit.end())
                .waitSeconds(0.5)
                .back(0.2,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .back(0.1,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();

        TrajectorySequence pos2_park = drive.trajectorySequenceBuilder(pos2_traj2.end())
                .back(5)
                //.strafeRight(27)
                //.forward(15)
                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
                .lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf

                .build();

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);


        waitForStart();
        for (int i = 0; i < 10; i++) {
            telemetryTfod();

        }
        if (dropPos1 > dropPos2 && dropPos1 > dropPos3) {
            finalDropPos = 1;

        } else if (dropPos2 > dropPos1 && dropPos2 > dropPos3) {
            finalDropPos = 2;
        } else if (dropPos3 > dropPos1 && dropPos3 > dropPos2) {
            finalDropPos = 3;
        } else {
            finalDropPos = 3;
            telemetry.addData("Failsafe Initiated: Robot going to DropPos: ", finalDropPos);
        }
        telemetry.addData("Final Pixel Position: ", finalDropPos);
        telemetry.update();

        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        sleep(500);
        setManualExposure(6, 250);
        targetFound = false;// Desired turning power/speed (-1 to +1)


        telemetry.addData("Final Pixel Position: ", finalDropPos);

        telemetry.update();

        if (!isStopRequested()) {

            if (finalDropPos == 1) {

                drive.followTrajectorySequence(pos1);

                useGrabber();
                sleep(waitTime+1500);
                setGrabber();

                drive.followTrajectorySequence(pos1_intake);

                while (!pixelCheck) {

                    if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                            (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                        pixelCheck = true;
                    }
                    if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2 && pixels == 1)){
                        pixelCheck = true;
                    }
                }

                //Lowering the Slides
                drive.followTrajectorySequence(pos1_deposit);
                CloseBox();
                IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime + 300);
                HoldSlides();
                BoardDropBox();

                forwardRobot();
                sleep(1000);

                if((((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5) && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) ){
                    dropBox();
                }

                drive.followTrajectorySequence(pos1_depositp2);

                stopRobot();
                sleep(500);
                OpenBox();
                sleep(200);

                drive.followTrajectorySequence(pos1_traj2);
                IntakeBox();

                SlideDown();

                sleep(300);

                drive.followTrajectorySequence(pos1_park);
                /*

                //Cycle 1
                drive.followTrajectorySequence(pos1_cycle);
                pixels = 0;



                drive.followTrajectorySequence(pos1_intake);

                while (!pixelCheck) {

                    if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                            (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                        pixelCheck = true;
                    }
                    if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2 && pixels == 1)){
                        pixelCheck = true;
                    }
                }

                drive.followTrajectorySequence(pos1_deposit);

                CloseBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                BoardDropBox();
                HoldSlides();


                drive.followTrajectorySequence(traj2);


                // drive.followTrajectorySequence(pos1_cycle_adjust); //move to left a little for deposition



                drive.followTrajectorySequence(park);


                DESIRED_TAG_ID = 1;
                strafeLeft();


                while (aprilAdjust) {
                    targetFound = false;
                    desiredTag = null;

                    // Step through the list of detected tags and look for a matching tag
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    // Tell the driver what we see, and what to do.
                    if (targetFound) {

                        strafeLeft();
                        if (desiredTag.ftcPose.bearing < -8) {
                            aprilAdjust = false;
                        }

                    }
                    telemetry.update();

                    // Apply desired axes motions to the drivetrain.
                    // moveRobot(drive, turn);
                    sleep(10);


                }


                stopRobot();

                forwardRobot();
                sleep(300); //600
                stopRobot();

                OpenBox();
                sleep(waitTimev3);

                backwardRobot();
                sleep(300);
                IntakeBox();
                stopRobot();
                */


            }

            if (finalDropPos == 2) {

                drive.followTrajectorySequence(pos2);

                useGrabber();
                sleep(waitTime+1500);
                setGrabber();

                drive.followTrajectorySequence(pos2_intake);

                drive.followTrajectorySequence(pos2_deposit);

                if((((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5) && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) ){
                    dropBox();
                }
                drive.followTrajectorySequence(pos2_depositp3);

                OpenBox();
                sleep(300);
                SlidePower(slidePower);
                sleep(400);
                IntakeBox();
                drive.followTrajectorySequence(pos2_traj2);

                SlideDown();
                sleep(300);

                drive.followTrajectorySequence(pos2_park);


            }


            //drive.followTrajectorySequence(pos2_depositp3);




/*

                DESIRED_TAG_ID = 2;
                strafeLeft();


                while (aprilAdjust) {
                    targetFound = false;
                    desiredTag = null;

                    // Step through the list of detected tags and look for a matching tag
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    // Tell the driver what we see, and what to do.
                    if (targetFound) {

                        strafeLeft();
                        if (desiredTag.ftcPose.bearing < -10) {
                            aprilAdjust = false;
                        }

                    }
                    telemetry.update();

                    // Apply desired axes motions to the drivetrain.
                    // moveRobot(drive, turn);
                    sleep(10);


                }


                stopRobot();

                forwardRobot();
                sleep(300); //600
                stopRobot();

                OpenBox();
                sleep(waitTimev3);

                backwardRobot();
                sleep(300);
                IntakeBox();
                stopRobot();

 */


        }




        if (finalDropPos == 3) {


            drive.followTrajectorySequence(pos3);

            useGrabber();
            sleep(waitTime+1500);
            setGrabber();

            drive.followTrajectorySequence(pos3_intake);



            while (!pixelCheck) {

                if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                        (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                    pixelCheck = true;
                }
                if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2 && pixels == 1)){
                    pixelCheck = true;
                }
            }
/*
            if((((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5) && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) ){
                dropBox();
            }

 */


            //Lowering the Slides
            drive.followTrajectorySequence(pos3_deposit);
            CloseBox();
            IntakeBox();
            SlidePower(slidePower);
            sleep(waitTime + 300);
            HoldSlides();
            BoardDropBox();

            drive.followTrajectorySequence(pos3_depositp2);

            if((((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5) && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) ){
                dropBox();
            }



            drive.followTrajectorySequence(pos3_depositp3);
            stopRobot();
            sleep(500);
            OpenBox();



            drive.followTrajectorySequence(pos3_traj2);
            IntakeBox();

            SlideDown();

            sleep(300);

            drive.followTrajectorySequence(pos3_park);


/*
            //Cycle 1
            drive.followTrajectorySequence(pos1_cycle);


            pixels = 0;



            drive.followTrajectorySequence(pos1_intake);

            while (!pixelCheck) {

                if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                        (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                    pixelCheck = true;
                }
                if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2 && pixels == 1)){
                    pixelCheck = true;
                }
            }

            drive.followTrajectorySequence(pos1_deposit);

            CloseBox();
            SlidePower(slidePower + 0.2);
            sleep(waitTime-250); //Change
            BoardDropBox();
            HoldSlides();

            drive.followTrajectorySequence(traj2);


            drive.followTrajectorySequence(pos1_cycle_adjust); //move to left a little for deposition



            drive.followTrajectorySequence(pos1_cycle_park);


            DESIRED_TAG_ID = 3;
            strafeLeft();


            while (aprilAdjust) {
                targetFound = false;
                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {

                    strafeLeft();
                    if (desiredTag.ftcPose.bearing < -8) {
                        aprilAdjust = false;
                    }

                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                // moveRobot(drive, turn);
                sleep(10);


            }


            stopRobot();

            forwardRobot();
            sleep(300); //600
            stopRobot();

            OpenBox();
            sleep(waitTimev3);

            backwardRobot();
            sleep(300);
            IntakeBox();
            stopRobot();

             */


        }

    }

    private void SlideDown(){
        boolean temp = true;

        while(temp){
            SlidePower(-0.4);
            if(LimitSwitch.isPressed()){
                telemetry.addData("Is Pressed", "HI");
                telemetry.update();
                temp = false;

            }
        }
    }

    private void dropBox(){
        BucketHold.setPosition(0.53);

        sleep(110);
        BucketHold.setPosition(0.7);
        sleep(750);

    }


    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");

        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");

        bleftDrive = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive = hardwareMap.get(DcMotor.class, "brm");
        rightDrive = hardwareMap.get(DcMotor.class, "blm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);
        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");

        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        GrabRoller = hardwareMap.get(CRServo.class, "GrabRoller");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketR.setDirection(Servo.Direction.REVERSE);


        Color = hardwareMap.get(NormalizedColorSensor.class, "Color");
        ColorFront = hardwareMap.get(NormalizedColorSensor.class, "Color2");
        RampSensor = hardwareMap.get(NormalizedColorSensor.class, "RampSensor");


        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Grabber = hardwareMap.get(Servo.class, "Grab");
        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");
        IntakeBox();



    }

    private void timeOut(){
        boolean switcheroo = true;
        long timeOut = System.currentTimeMillis();
        while (switcheroo && timeOut < (System.currentTimeMillis() + 5000)) { // Loop while switcheroo is true and 5 seconds have not passed
            if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) < 4)) {
                Intake.setPower(1);
                IntakeRoller.setPower(-0.8);
                switcheroo = false;
            } else if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) > 4)) {
                break;
            }
        }
    }

    private void SlidePower(double p){
        SlideR.setPower(-p);
        SlideL.setPower(p);

    }

    private void useGrabber(){
        Grabber.setPosition(0.81);//Change for taking 1 pixel 5
        GrabRoller.setPower(1);

    }

    private void useGrabbercycle(){
        Grabber.setPosition(0.85);
        GrabRoller.setPower(1);

    }



    private void setGrabber(){
        Grabber.setPosition(0.47);
        GrabRoller.setPower(0);
    }

    private void ReleasePixel() {
        PixelPusher.setPosition(0.15);
    }

    private void CloseBox(){
        //BucketHold.setPosition(0); //close
        BucketHold.setPosition(0.7);
    }
    private void OpenBox(){
        BucketHold.setPosition(0.5); //close


    }
    private void BoardDropBox(){
        BucketL.setPosition(0.955);
        BucketR.setPosition(0.955);
    }

    private void HoldSlides(){
        SlideR.setPower(-0.1);
        SlideL.setPower(0.1);
    }

    private void StopSlides(){
        SlideR.setPower(0);
        SlideL.setPower(0);
    }
    private void IntakeBox(){
        BucketL.setPosition(0.63);
        BucketR.setPosition(0.63);
//        BucketR.setPosition(0.59);
//        BucketL.setPosition(0.63);
    }

/*
      if (LimitSwitch.isPressed()) {
        telemetry.addData("Intake", "Ready");
        if(Rumbled = false){
            gamepad2.rumble(200);
            Rumbled= true;
        }
 */

    /**
     * Initialize the TensorFlow Object Detection processor.
     */



    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();
        aprilTag.setDecimation(2);
        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();
        tfod.setMinResultConfidence(0.75f);
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    public void strafeLeft(){
        leftDrive.setPower(-0.52);//-0.52
        bleftDrive.setPower(0.5);//0.5
        rightDrive.setPower(0.54);//0.54
        brightDrive.setPower(-0.5);//-0.5
    }

    public void strafeLeftSped(){
        leftDrive.setPower(-0.98);//-0.52
        bleftDrive.setPower(1);//0.5
        rightDrive.setPower(0.96);//0.54
        brightDrive.setPower(-1);//-0.5
    }

    public void stopRobot(){
        leftDrive.setPower(0);
        bleftDrive.setPower(0);
        rightDrive.setPower(0);
        brightDrive.setPower(0);
    }

    public void forwardRobot(){
        leftDrive.setPower(0.34);
        bleftDrive.setPower(0.34);
        rightDrive.setPower(0.34);
        brightDrive.setPower(0.34);
    }

    public void backwardRobot(){

        leftDrive.setPower(-0.34);
        bleftDrive.setPower(-0.34);
        rightDrive.setPower(-0.34);
        brightDrive.setPower(-0.34);
    }

    public void resetSpeed(){
        leftDrive.setPower(0.1);
        bleftDrive.setPower(0.1);
        rightDrive.setPower(0.1);
        brightDrive.setPower(0.1);
    }

    private void DownLimit(){
        while (!LimitSwitch.isPressed()){
            SlideR.setPower(-0.1);
            SlideL.setPower(0.1);
        }
    }




    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            //telemetry.addData("X value: ", x);


            if(x>=0 && x<=200){
                dropPos1 ++;
                telemetry.addData("Pixel Position :", "dropPos1");
            }
            if(x>=201 && x<=700){
                dropPos2 ++;
                telemetry.addData("Pixel Position:", "drop");
            }
            if(x>=891 && x<=900){
                dropPos3 ++;
                telemetry.addData("Pixel Position:", "dropPos3");
            }
        }   // end for() loop

    }   // end method telemetryTfod()



}