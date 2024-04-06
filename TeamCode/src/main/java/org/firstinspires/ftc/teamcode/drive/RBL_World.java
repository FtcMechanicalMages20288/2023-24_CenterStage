package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(group = "RBL_World")
public class RBL_World extends LinearOpMode {

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












    // Tfod Stuff Above

    private Servo BucketHold, BucketR, BucketL, PixelPusher;
    private DcMotor SlideR, SlideL, Intake;
    private CRServo GrabRoller;

    private Servo Grabber;
    private CRServo IntakeRoller;


    public static double slidePower = 0.45;
    int xValue = 19;
    int yValue = -10;

    public static int x2Value = 22;
    public static int y2Value = 4 ;

    public static int x22Value = 19;
    public static int y22Value = 25;

    public static int turn1 = 100;
    public static int turn2 = 85;

    public static int turn3_3 = -55;
    public static int turn3_5 = 95;

    public static int strafetox = 0;

    public static int strafetoy = 28;


    public static int x3Value = 28;
    public static int y3Value = -5;

    public static int x33Value = 25;
    public static int y33Value = 25;


    public static int x4Value = 20;
    public static int y4Value = 0 ;

    public static int x44Value = 34;
    public static int y44Value = 25;



    public static double x3Value2 = -22;
    public static int y3Value2 = 34 ;

    public static int bw = 10;



    public static int waitTime = 610;
    public static int waitTimev2 = 750;

    public static double FwBw = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();
        initTfod();
      //  CloseBox();
        PixelPusher.setPosition(0.45); // locks pixel




        Pose2d sP = new Pose2d(0,0,0);

        drive.setPoseEstimate(sP);








        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(24, 6))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(800);
                })
                .back(9)
                .lineToLinearHeading(new Pose2d(x22Value,y22Value,Math.toRadians(90)))
                .build();



        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(pos1.end())
                .forward(FwBw+3,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();


        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
              /*  .addDisplacementMarker(() -> {
                    IntakeBox();
                })*/
                .lineToLinearHeading(new Pose2d(5, 10, Math.toRadians(90)))
                //  .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(5,10,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(15, -68, Math.toRadians(135)))

                .build();

        TrajectorySequence traj4_p2 = drive.trajectorySequenceBuilder(traj4.end())
                .addDisplacementMarker(() ->{
                    useGrabber();
                    GrabRoller.setPower(0.85);


                })
                .build();

        TrajectorySequence traj4_p3 = drive.trajectorySequenceBuilder(traj4.end())
                .addDisplacementMarker(() ->{

                    Intake.setPower(0.8);
                    IntakeRoller.setPower(-0.8);
                    sleep(1000);
                    Intake.setPower(0);
                    IntakeRoller.setPower(0);


                })
                .build();


        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(5,10,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(28,25, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();




        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(x3Value, y3Value, Math.toRadians(-5))) // added to make robot more straight
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(800);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(x33Value,y33Value+1,Math.toRadians(turn2+5))) //90
                .build();



        TrajectorySequence traj2_2 = drive.trajectorySequenceBuilder(pos2.end())
                .forward(FwBw+1,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();


        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(traj2_2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .lineToLinearHeading(new Pose2d(strafetox+3,strafetoy,Math.toRadians(90)))
                .build();

        TrajectorySequence traj2_4 = drive.trajectorySequenceBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(15, -77, Math.toRadians(140)))
                .build();

        TrajectorySequence traj2_5 = drive.trajectorySequenceBuilder(traj2_3.end())
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(5,10,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(28,25, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();




        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(x4Value, y4Value))
                .turn(Math.toRadians(turn3_3))
                .forward(10)
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(800);
                })
                .back(14)
                // .back(4)
                // .turn(Math.toRadians(turn3_5))
                .lineToLinearHeading(new Pose2d(x44Value-2,y44Value,Math.toRadians(85)))
                .build();



        TrajectorySequence traj3_2 = drive.trajectorySequenceBuilder(pos3.end())
                .forward(FwBw+3,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )

                .build();


        TrajectorySequence traj3_3 = drive.trajectorySequenceBuilder(traj3_2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    IntakeBox();
                })
                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();

        TrajectorySequence traj3_5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(5,10,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(35,25, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();








        waitForStart();

        telemetryTfod();
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

        if(!isStopRequested()){
            if(finalDropPos == 1 ) {
                drive.followTrajectorySequence(pos1);

               /* IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime+210);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);*/
                drive.followTrajectorySequence(traj2);
                /*OpenBox();
                sleep(waitTimev2);*/
               // drive.followTrajectorySequence(traj3);
                drive.followTrajectorySequence(traj4);
                drive.followTrajectorySequence(traj5);


            }
            if(finalDropPos == 2 ) {
                drive.followTrajectorySequence(pos2);

             /*   IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime+210);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);*/
                drive.followTrajectorySequence(traj2_2);
               /* OpenBox();
                sleep(waitTimev2);*/

               // drive.followTrajectorySequence(traj2_3);
                drive.followTrajectorySequence(traj4);
                drive.followTrajectorySequence(traj5);



            }
            if(finalDropPos == 3) {

                drive.followTrajectorySequence(pos3);

              /*  IntakeBox();
                SlidePower(slidePower);
                sleep(waitTime+210);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2); */
                drive.followTrajectorySequence(traj3_2);
               /* OpenBox();
                sleep(waitTimev2); */
             // drive.followTrajectorySequence(traj3_3);
                drive.followTrajectorySequence(traj4);
                drive.followTrajectorySequence(traj3_5);





            }

        }




    }
    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");



        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");

        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketR.setDirection(Servo.Direction.REVERSE);
        Grabber = hardwareMap.get(Servo.class, "Grab");
        GrabRoller = hardwareMap.get(CRServo.class, "GrabRoller");
        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");
        setGrabber();



    }

    private void SlidePower(double p){
        SlideR.setPower(p);
        SlideL.setPower(p);





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
    private void setGrabber(){
        Grabber.setPosition(0.15);
    }

    private void useGrabber(){
        Grabber.setPosition(0.55);
    }
    private void dropBox(){
        BucketHold.setPosition(0.05);
        sleep(100);
        BucketHold.setPosition(0.15);
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
        BucketR.setPosition(0.61);
        BucketL.setPosition(0.65);
//        BucketR.setPosition(0.59);
//        BucketL.setPosition(0.63);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

     /*   // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
*/
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }



        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        //builder.addProcessor(aprilTag);
        builder.addProcessor(tfod);



        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()



    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */


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


            if(x>=0 && x<=300){
                dropPos1 ++;
                telemetry.addData("Pixel Position :", "dropPos1");
            }
            if(x>=301 && x<=891){
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