package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(group = "RRL Final")
public class RRLFinal extends LinearOpMode {

    int dropPos1;
    int dropPos2;
    int dropPos3;
    int finalDropPos;

    // private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    // private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    // private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    // boolean targetFound = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedElement.tflite";
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
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)


    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    int DESIRED_TAG_ID = 3;   // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound = false;



    boolean aprilAdjust = true;





    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor  brightDrive = null;
    private DcMotor bleftDrive = null;

    private ElapsedTime runtime = new ElapsedTime();


    private VisionPortal visionPortal1;









    // Tfod Stuff Above

    private Servo BucketHold, BucketR, BucketL;
    private DcMotor SlideR, SlideL, Intake;

    public static double slidePower = 0.45;
    int xValue = 19;
    int yValue = -10;

    public static int x2Value = 26;
    public static int y2Value = 4 ;


    public static int x3Value = 23;
    public static int y3Value = 5;

    public static double x3Value2 = -22;
    public static int y3Value2 = 34 ;

    public static int bw = 10;

    public static int turn3 = 110;

    public static int waitTime = 700;
    public static int waitTimev2 = 750;

    public static double FwBw = 8;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();
        initDoubleVision();
        sleep(3000);
        CloseBox();




        Pose2d sP = new Pose2d(0,0,0);

        Pose2d AprilAdjust = new Pose2d(0,0,0);

        drive.setPoseEstimate(sP);







        //pos1
        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)

                .lineToLinearHeading(new Pose2d(20, 8))
                .back(7)

                // akash's add ons (after pixel drop)
                .lineTo(new Vector2d(18, -6))
                .lineTo(new Vector2d(51, -2))
                .turn(Math.toRadians(-94))

                //driving thru the stage door
                .lineTo(new Vector2d(49, -70))

                .build();

        /***************************************************/
        //pos1 --->

        //pos1
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(pos1.end())
                .forward(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();




        //pos1
        TrajectorySequence pos1p3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .lineToLinearHeading(new Pose2d(0,28,Math.toRadians(90)))
                .build();

        /***************************************************/
        //pos2 --->

        //pos2
        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                /*.lineToLinearHeading(new Pose2d(28, -2))
                .back(5)
                .strafeLeft(6)
                .forward(12)

                .lineToLinearHeading(new Pose2d(-2,51,Math.toRadians(-95)))
                .build();*/

                .lineToLinearHeading(new Pose2d(30, 0))
                .back(7)
                .lineToLinearHeading(new Pose2d(30,-14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(53,-14, Math.toRadians(70)))
                //.turn(Math.toRadians(90))
                //.lineTo(new Vector2d(53, 80))
                //.lineToLinearHeading(new Pose2d(53,80, Math.toRadians(70)))
                //.turn(Math.toRadians(200))
                .forward(90)

                .build();


        //pos2
        TrajectorySequence traj2_2 = drive.trajectorySequenceBuilder(pos2.end())
                .forward(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();

        //pos2
        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(traj2_2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .lineToLinearHeading(new Pose2d(3,28,Math.toRadians(90)))
                .build();

        /***************************************************/
        //pos3 --->


        //pos3
        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(sP)

                .lineTo(new Vector2d(20, -2))
                .turn(Math.toRadians(-50))
                .forward(6)
                .back(8)
                .back(7)
                .turn(Math.toRadians(50))

                // akash's add ons (after pixel push)
                .lineTo(new Vector2d(18, 0))
                .lineTo(new Vector2d(52, -2))
                .turn(Math.toRadians(-95)) //
                .lineTo(new Vector2d(45, -70)) // 52

                .build();

        //pos3
        TrajectorySequence traj2v3 = drive.trajectorySequenceBuilder(pos3.end())
                .forward(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();
        //pos3
        TrajectorySequence traj3v3 = drive.trajectorySequenceBuilder(traj2v3.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .lineToLinearHeading(new Pose2d(5,28,Math.toRadians(90)))
                .build();











        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);

        while (opModeInInit()) {
            telemetryTfod();
            telemetry.addData(" Pixel Position: ", finalDropPos);
            telemetry.update();
        }


        waitForStart();
        //finalDropPos = 3;
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        telemetry.addData("Final Pixel Position: ", finalDropPos);
        telemetry.update();
        sleep(3000);
        setManualExposure(6, 250);
        targetFound = false;// Desired turning power/speed (-1 to +1) +ve is CounterClockwise


        telemetry.addData("Final Pixel Position: ", finalDropPos);

        telemetry.update();

        if(!isStopRequested()){

            if(finalDropPos == 1 ) {

                drive.followTrajectorySequence(pos1);

                IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);

                DESIRED_TAG_ID = 1;
                strafeRight();


                while(aprilAdjust) {
                    targetFound = false;
                    desiredTag  = null;

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

                        strafeRight();
                        if(desiredTag.ftcPose.bearing > 15){
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
                sleep(1000);
                stopRobot();

                OpenBox();
                sleep(waitTimev2);

                backwardRobot();
                sleep(400);
                stopRobot();

                IntakeBox();
                sleep(500);
                stopRobot();

            }

            if(finalDropPos == 2 ) {

                drive.followTrajectorySequence(pos2);

                IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);

                DESIRED_TAG_ID = 2;
                strafeRight();


                while(aprilAdjust) {
                    targetFound = false;
                    desiredTag  = null;

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

                        strafeRight();
                        if(desiredTag.ftcPose.bearing < -15){
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
                sleep(1000);
                stopRobot();

                OpenBox();
                sleep(waitTimev2);

                backwardRobot();
                sleep(400);
                IntakeBox();
                sleep(500);
                stopRobot();

            }
            if(finalDropPos == 3) {


                drive.followTrajectorySequence(pos3);

                IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);

                DESIRED_TAG_ID = 3;
                strafeRight();


                while(aprilAdjust) {
                    targetFound = false;
                    desiredTag  = null;

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

                        strafeRight();
                        if(desiredTag.ftcPose.bearing < -10){
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
                sleep(500);
                stopRobot();

                OpenBox();
                sleep(waitTimev2);

                backwardRobot();
                sleep(500);
                IntakeBox();
                sleep(500);
                stopRobot();


            }

        }




    }
    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");


        bleftDrive  = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive  = hardwareMap.get(DcMotor.class, "brm");
        rightDrive = hardwareMap.get(DcMotor.class, "blm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);

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
        BucketHold.setPosition(0); //close
    }
    private void OpenBox(){
        BucketHold.setPosition(0.7); //close
    }
    private void BoardDropBox(){
        BucketR.setPosition(0.15);
        BucketL.setPosition(0.15);
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
        leftDrive.setPower(-0.5);
        bleftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        brightDrive.setPower(-0.5);
    }

    public void strafeRight(){
        leftDrive.setPower(0.5);
        bleftDrive.setPower(-0.5);
        rightDrive.setPower(-0.5);
        brightDrive.setPower(0.5);
    }

    public void stopRobot(){
        leftDrive.setPower(0);
        bleftDrive.setPower(0);
        rightDrive.setPower(0);
        brightDrive.setPower(0);
    }

    public void forwardRobot(){
        leftDrive.setPower(0.25);
        bleftDrive.setPower(0.25);
        rightDrive.setPower(0.25);
        brightDrive.setPower(0.25);
    }

    public void backwardRobot(){

            leftDrive.setPower(-0.25);
            bleftDrive.setPower(-0.25);
            rightDrive.setPower(-0.25);
            brightDrive.setPower(-0.25);
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

            if(x>=0 && x<=300){
                finalDropPos = 1;
                telemetry.addData("Pixel Position :", "dropPos1");
            }
            if(x>=301 && x<=891){
                finalDropPos = 2;
                telemetry.addData("Pixel Position:", "dropPos2");
            }
            if(x>=891 && x<=900){
                finalDropPos = 3;
                telemetry.addData("Pixel Position:", "dropPos3");
            } else {
                finalDropPos = 3;
                telemetry.addData("Detection Failed. Default Pixel Position:", "dropPos3");
            }
        }   // end for() loop

    }   // end method telemetryTfod()

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        leftDrive.setPower(leftPower);
        bleftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        brightDrive.setPower(rightPower);
    }

    }   // end method telemetryTfod()



