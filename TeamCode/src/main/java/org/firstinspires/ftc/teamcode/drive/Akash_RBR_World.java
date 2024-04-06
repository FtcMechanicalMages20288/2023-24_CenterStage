package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
@Autonomous(group = "Akash_RBR_World")
public class Akash_RBR_World extends LinearOpMode {

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
    private static final String TFOD_MODEL_ASSET = "BlueElement.tflite";
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
    private CRServo GrabRoller;


    private TouchSensor LimitSwitch;

    private Servo Grabber;

    private CRServo IntakeRoller;






    // Tfod Stuff Above

    private Servo BucketHold, BucketR, BucketL, PixelPusher;
    private DcMotor SlideR, SlideL, Intake;

    public static double slidePower = 0.45;
    int xValue = 19;
    int yValue = -10;

    public static int x2Value = 26;
    public static int y2Value = 4 ;

    public static int pos2startx = 30;
    public static int pos2starty = 2 ;

    public static int strafecorrect = 8 ;

    public static int fwdDistancep1 = 30;
    public static int fwdDistancep2 = 50;

    public static int cycleX = 30;
    public static int cycleY = 30;




    public static double DISTANCE = 8; // in

    public static int waitTime = 700;
    public static int waitTimev2 = 750;
    public static int waitTimev3 = 1000;

    public static double FwBw = 8;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();
        initDoubleVision();
        sleep(3000);
        CloseBox();
        PixelPusher.setPosition(0.45);
        Grabber.setPosition(0.15);







        Pose2d sP = new Pose2d(0,0,0);
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
                .lineToLinearHeading( new Pose2d(48, 0.5))
                .turn(Math.toRadians(84))

                //.turn(Math.toRadians(-88))
                .lineToConstantHeading(new Vector2d(55, 70))

                .addDisplacementMarker(() -> {

                    IntakeBox();
                    SlidePower(slidePower);
                    sleep(waitTime+500); //higher 500
                    HoldSlides();
                    BoardDropBox();
                    // sleep(waitTimev2);

                })
                .lineTo(new Vector2d(54, 75))
                /* .addDisplacementMarker(() -> {
                     OpenBox();
                     sleep(1000);
                     backwardRobot();
                     sleep(500);
                     stopRobot();
                 })
                 .waitSeconds(1.6)*/
                //.turn(Math.toRadians(200))
                //.turn(Math.toRadians(-15.5))


                .build();



        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)


                .lineToLinearHeading(new Pose2d(20, 0))
                .turn(Math.toRadians(50-11))
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
                .lineToLinearHeading(new Pose2d(49, -14, Math.toRadians(90)))

                .build();

        TrajectorySequence pos1_p2 = drive.trajectorySequenceBuilder(pos1.end())




                .lineToConstantHeading(new Vector2d(51, 75))


                .addDisplacementMarker(()->{
                    setGrabber();

                })
                .addDisplacementMarker(()->{

                    Intake.setPower(-1);
                    IntakeRoller.setPower(0.8);

                })

                /* .addDisplacementMarker(() -> {

                     IntakeBox();
                     SlidePower(slidePower);
                     sleep(waitTime+500);
                     HoldSlides();
                     BoardDropBox();
                     // sleep(waitTimev2);

                 })*/
                //.lineTo(new Vector2d(54, 75))
                /*.addDisplacementMarker(() -> {
                    OpenBox();
                    sleep(1000);
                    backwardRobot();
                    sleep(500);
                    stopRobot();
                })
                .waitSeconds(1.6)*/
                //.turn(Math.toRadians(200))
                //.turn(Math.toRadians(-15.5))
                .build();

        TrajectorySequence pos1_p3 = drive.trajectorySequenceBuilder(pos1.end())
                .addDisplacementMarker(() -> { //Change Duration
                    // Run action at 2.5 seconds into the path (1.5 seconds after the previous marker)
                    useGrabber();
                    GrabRoller.setPower(0.95);
                    Intake.setPower(.8);
                    IntakeRoller.setPower(-0.8);


                })
                .addDisplacementMarker(() -> {
                    sleep(3000);
                })
                .addDisplacementMarker(() -> {//Change Duration

                    Intake.setPower(.8);
                    IntakeRoller.setPower(-0.8);
                    setGrabber();
                    GrabRoller.setPower(0);



                })
                .back(0.3)
                .addDisplacementMarker(() -> {
                    
                    sleep(1000);
                })
                .addDisplacementMarker(() -> { //After First Temporal Marker: Outake Any pixels
                    Intake.setPower(-1);
                    IntakeRoller.setPower(0.8);

                })

                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(pos2startx, pos2starty))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                .back(4)
                .turn(Math.toRadians(88))


                .lineToLinearHeading(new Pose2d(26, 32, Math.toRadians(88)))
                .waitSeconds(4) // Change depending on teammate speed
                .lineToLinearHeading(new Pose2d(25.5, 75, Math.toRadians(88)))
                .addSpatialMarker(new Vector2d(25.7,73) , ()-> {
                    IntakeBox();
                    SlidePower(slidePower);
                    sleep(waitTime+400);
                    HoldSlides();
                    BoardDropBox();
                })

                .waitSeconds(0.3)
                .strafeRight(.5)

                .forward(14,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )




                .build();










        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);



        waitForStart();
        for (int i = 0; i <  10; i++) {
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
        targetFound = false;// Desired turning power/speed (-1 to +1) +ve is CounterClockwise


        telemetry.addData("Final Pixel Position: ", finalDropPos);

        telemetry.update();

        if(!isStopRequested()){

            if(finalDropPos == 1 ) {
                //orginal krrish verson
                drive.followTrajectorySequence(pos1);
                /*
                .lineToLinearHeading(new Pose2d(20, 0))
                        .turn(Math.toRadians(50-11))
                        .forward(8)
                        .addDisplacementMarker(() -> {
                            ReleasePixel();
                        })
                        .addDisplacementMarker(() -> {
                            sleep(500);
                        })
                        .back(8)

                        //.lineTo(new Vector2d(18, 2))
                        .lineToLinearHeading(new Pose2d(48, -2, Math.toRadians(85)))
                        .back(17)

                        .build();

                 */
                /*
                forwardRobot();
                sleep(1500);
                useGrabber();
                Intake.setPower(1);
                IntakeRoller.setPower(-0.8);
                sleep(100);
                backwardRobot();
                sleep(1500);
                drive.followTrajectorySequence(pos1_p2);
                */
                drive.followTrajectorySequence(pos1_p3);
                // sleep(1000);
                // backwardRobot();
                // sleep(500);
                drive.followTrajectorySequence(pos1_p2);




                DESIRED_TAG_ID = 1;
                strafeLeft();


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

                        strafeLeft();
                        if(desiredTag.ftcPose.bearing < -8){
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
                sleep(600);
                stopRobot();

                OpenBox();
                sleep(waitTimev3);

                backwardRobot();
                sleep(300);
                stopRobot();
                IntakeBox();



            }

            if(finalDropPos == 2 ) {

                drive.followTrajectorySequence(pos2);

                OpenBox();
                sleep(1000);
                backwardRobot();
                sleep(500);
                stopRobot();
                IntakeBox();


            }
            if(finalDropPos == 3) {


                drive.followTrajectorySequence(pos3);


                DESIRED_TAG_ID = 3;
                strafeLeft();


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

                        strafeLeft();
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
                sleep(1000);
                stopRobot();

                OpenBox();
                sleep(waitTimev3);

                backwardRobot();
                sleep(300);
                IntakeBox();
                sleep(250);
                stopRobot();

            }

        }



    }
    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");

        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");

        bleftDrive  = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive  = hardwareMap.get(DcMotor.class, "brm");
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


        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Grabber = hardwareMap.get(Servo.class, "Grab");
        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");
    }
    private void SlidePower(double p){
        SlideR.setPower(p);
        SlideL.setPower(p);

    }
    private void useGrabber(){
        Grabber.setPosition(0.55);

    }

    private void setGrabber(){
        Grabber.setPosition(0.15);
    }

    private void ReleasePixel() {
        PixelPusher.setPosition(0.15);
    }

    private void CloseBox(){
        //BucketHold.setPosition(0); //close
        BucketHold.setPosition(0.15);
    }
    private void OpenBox(){
        //  BucketHold.setPosition(0.7); //close



        BucketHold.setPosition(0.05);

    }
    private void BoardDropBox(){
        BucketR.setPosition(0.23);
        BucketL.setPosition(0.27);
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
        BucketR.setPosition(0.58);
        BucketL.setPosition(0.62);
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
        leftDrive.setPower(-0.52);
        bleftDrive.setPower(0.5);
        rightDrive.setPower(0.54);
        brightDrive.setPower(-0.5);
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
                telemetry.addData("Pixel Position:", "dropPos2");
            }
            if(x>=891 && x<=900){
                dropPos3 ++;
                telemetry.addData("Pixel Position:", "dropPos3");
            }
        }   // end for() loop

    }   // end method telemetryTfod()



}