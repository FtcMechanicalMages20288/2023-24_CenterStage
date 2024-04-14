package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.objdetect.Board;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(group = "RRR_World")
public class RRR_World extends LinearOpMode {

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












    // Tfod Stuff Above

    private Servo BucketHold, BucketR, BucketL, PixelPusher;
    private DcMotor SlideR, SlideL, Intake;
    private CRServo GrabRoller;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor brightDrive = null;
    private DcMotor bleftDrive = null;

    private TouchSensor LimitSwitch;
    private Servo Grabber;
    private CRServo IntakeRoller;

    private NormalizedColorSensor RampSensor, Color, ColorFront;

    int pixels = 1;
    boolean switcheroo = true;

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
    public static int WaitageTimeVar = 0; //Time Displacement Variable

    public static double FwBw = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initCode();
        CloseBox();
        initTfod();
        PixelPusher.setPosition(0.45); // locks pixel




        Pose2d sP = new Pose2d(0,0,0);

        drive.setPoseEstimate(sP);








        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)

                .lineToLinearHeading(new Pose2d(x4Value, y4Value))
                .turn(Math.toRadians(50-11))
                .forward(8)
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                .back(8)
                .back(7)

                //goes to board
                .turn(Math.toRadians(-90+11))
                .lineToLinearHeading(new Pose2d(33,-23,Math.toRadians(-90)))


                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(pos1.end())
                .forward(FwBw+4.5,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    OpenBox();
                    sleep(200);
                    SlidePower(slidePower + 0.2);
                    sleep(250);
                    IntakeBox();
                    sleep(100);
                    HoldSlides();
                    StopSlides();
                    //sleep(300);
                })


                .build();






        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(FwBw,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                /*  .addDisplacementMarker(() -> {
                      IntakeBox();
                  })
                  .lineToLinearHeading(new Pose2d(5, 10, Math.toRadians(90)))*/
                .build();


        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())

                .addDisplacementMarker(() -> {
                    SlideDown();

                })

                .lineToLinearHeading(new Pose2d(4,-8,Math.toRadians(-90))) //x = 4.5 prev
                .lineToLinearHeading(new Pose2d(4, 58, Math.toRadians(-90))) //x = 4.5 prev
                .lineToLinearHeading(new Pose2d(16.5,65, Math.toRadians(-90-23)))
                //  .turn(Math.toRadians(-3.6))

                .back(6.5)

                .addDisplacementMarker(() -> {
                    useGrabber();
                })


                .build();
        TrajectorySequence intake = drive.trajectorySequenceBuilder(traj4.end())

                .waitSeconds(1.8)
                .addDisplacementMarker(() -> {
                    setGrabber();
                })
                .addDisplacementMarker(() -> {
                    sleep(2000);
                    IntakePix();
                })



                //.back(3.8)

                .waitSeconds(0.5)

                .addDisplacementMarker(() -> {

                    long timeOut = System.currentTimeMillis();

                    while (switcheroo && (timeOut + 1680) < (System.currentTimeMillis())) { // Loop while switcheroo is true and 5 seconds have not passed

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


                .build();



        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(intake.end())
//                .lineToLinearHeading(new Pose2d(4, -55, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(4,10,Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(28,25, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .waitSeconds(1.2)
                .addDisplacementMarker(() -> {
                    Intake.setPower(1);
                    IntakeRoller.setPower(-0.8);

                })
                // new code!!!!

                //.lineToLinearHeading(new Pose2d(15, -68, Math.toRadians(136)))

                .lineToLinearHeading(new Pose2d(4, 60, Math.toRadians(-90))) //x = 4.5 prev
                .lineToLinearHeading(new Pose2d(4,-8,Math.toRadians(-90))) //x = 4.5 prev


                .addDisplacementMarker(()->{
                    boolean pixelCheck = false;
                    boolean breakyCheck = false;
                    while (!pixelCheck) {

                        if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                                (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                            pixelCheck = true;
                        }
                        if ((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2 && pixels == 1)){
                            pixelCheck = true;
                        }
                        else{
                            breakyCheck = true;
                            pixelCheck = true;

                        }
                    }
                    IntakeRoller.setPower(0);
                    Intake.setPower(0);
                    CloseBox();



                })






                //Traj4
//                .lineToLinearHeading(new Pose2d(4,10,Math.toRadians(90))) //x = 4.5 prev
//                .lineToLinearHeading(new Pose2d(4, -55, Math.toRadians(90))) //x = 4.5 prev
//                .lineToLinearHeading(new Pose2d(15, -68, Math.toRadians(136)))
//                .forward(6)

                .build();

        TrajectorySequence traj5p2 = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(22,-23, Math.toRadians(-90)))
                .forward(FwBw+4.5,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(traj5p2.end())

                .back(FwBw)
                .lineToLinearHeading(new Pose2d(strafetox+5,-27,Math.toRadians(-90)))

                .build();


        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(29, -4))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(500);
                })
                .back(10)
                //going to board
                .lineToLinearHeading(new Pose2d(28,-25,Math.toRadians(-90)))
                .build();



        TrajectorySequence traj2_2 = drive.trajectorySequenceBuilder(pos2.end())
                .waitSeconds(0.5)
                .forward(FwBw+3,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    OpenBox();
                    sleep(200);
                    SlidePower(slidePower + 0.2);
                    sleep(400);
                    IntakeBox();
                    sleep(100);
                    HoldSlides();
                    StopSlides();
                    //sleep(300);
                })


                .build();









        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(sP)
                .lineToLinearHeading(new Pose2d(20, -13))
                .addDisplacementMarker(() -> {
                    ReleasePixel();
                })
                .addDisplacementMarker(() -> {
                    sleep(800);
                })
                .back(9)

                .lineToLinearHeading(new Pose2d(x22Value+0.5,-y22Value,Math.toRadians(-90)))

                .build();

        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(pos3.end())
                .forward(FwBw+3,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .addDisplacementMarker(() -> {
                    OpenBox();
                    sleep(200);
                    SlidePower(slidePower + 0.2);
                    sleep(400);
                    IntakeBox();
                    sleep(100);
                    HoldSlides();
                    StopSlides();
                    //sleep(300);
                })
                // .lineToLinearHeading(new Pose2d(strafetox+3,strafetoy,Math.toRadians(90)))
                .build();








        TrajectorySequence breakPark = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(strafetox+5,-33,Math.toRadians(-90)))
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

                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();
                drive.followTrajectorySequence(traj2);



                //drive.followTrajectorySequence(traj3);


                drive.followTrajectorySequence(traj4);
                //useGrabber();


                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);
                boolean pixelCheck = false;
                boolean breakerCheck = false;
                while (!pixelCheck) {

                    if (!(((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                            !(((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                        breakerCheck = true;
                        pixelCheck = true;
                    }
                    else{
                        pixelCheck = true;
                        telemetry.addData("check", "Dhruv sucks");
                        telemetry.update();
                    }
                }
                IntakeRoller.setPower(0);
                Intake.setPower(0);
                CloseBox();

                //sleep(5000);
                /*IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime+200);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);*/

                if(!breakerCheck) {

                    //sleep(5000);
                    SlidePower(slidePower + 0.2);
                    sleep(waitTime + 200);
                    HoldSlides();
                    BoardDropBox();
                    // sleep(waitTimev2);

                    drive.followTrajectorySequence(traj5p2);
                    OpenBox();
                    sleep(475);
                    IntakeBox();
                    sleep(300);
                    drive.followTrajectorySequence(park);
                } else if(breakerCheck){
                    drive.followTrajectorySequence(breakPark);
                }



            }
            if(finalDropPos == 2 ) {
                drive.followTrajectorySequence(pos2);

                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();
                drive.followTrajectorySequence(traj2_2);



                //drive.followTrajectorySequence(traj3);


                drive.followTrajectorySequence(traj4);
                //useGrabber();

                IntakePix();
                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);
                boolean pixelCheck = false;
                boolean breakerCheck = false;
                while (!pixelCheck) {

                    if (!(((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                            !(((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                        breakerCheck = true;
                        pixelCheck = true;
                    }
                    else{
                        pixelCheck = true;
                    }
                }
                IntakeRoller.setPower(0);
                Intake.setPower(0);
                CloseBox();

                //sleep(5000);
                /*IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime+200);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);*/

                if(!breakerCheck) {

                    //sleep(5000);
                    SlidePower(slidePower + 0.2);
                    sleep(waitTime + 200);
                    HoldSlides();
                    BoardDropBox();


                    drive.followTrajectorySequence(traj5p2);
                    OpenBox();
                    sleep(475);
                    IntakeBox();
                    sleep(300);
                    drive.followTrajectorySequence(park);
                } else if(breakerCheck){
                    drive.followTrajectorySequence(breakPark);
                }

            }
            if(finalDropPos == 3) {
                drive.followTrajectorySequence(pos3);

                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();
                drive.followTrajectorySequence(traj2_3);



                //drive.followTrajectorySequence(traj3);


                drive.followTrajectorySequence(traj4);
                //useGrabber();

                IntakePix();
                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);
                boolean pixelCheck = false;
                boolean breakerCheck = false;
                while (!pixelCheck) {

                    if (!(((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                            !(((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)) {
                        breakerCheck = true;
                        pixelCheck = true;
                    }
                    else{
                        pixelCheck = true;
                    }
                }
                IntakeRoller.setPower(0);
                Intake.setPower(0);
                CloseBox();

                //sleep(5000);
                /*IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime+200);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);*/

                if(!breakerCheck) {
                    //sleep(5000);
                    SlidePower(slidePower + 0.2);
                    sleep(waitTime + 200);
                    HoldSlides();
                    BoardDropBox();

                    drive.followTrajectorySequence(traj5p2);
                    OpenBox();
                    sleep(475);
                    IntakeBox();
                    sleep(300);
                    drive.followTrajectorySequence(park);
                } else if(breakerCheck){
                    drive.followTrajectorySequence(breakPark);
                }

            }

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
    private void initCode() {
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");

        bleftDrive = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive = hardwareMap.get(DcMotor.class, "brm");
        rightDrive = hardwareMap.get(DcMotor.class, "blm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);

        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");

        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");

        BucketL = hardwareMap.get(Servo.class, "BucketL");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
        BucketR = hardwareMap.get(Servo.class, "BucketR");
        BucketR.setDirection(Servo.Direction.REVERSE);

        Grabber = hardwareMap.get(Servo.class, "Grab");
        GrabRoller = hardwareMap.get(CRServo.class, "GrabRoller");
        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");
        Intake = hardwareMap.dcMotor.get("Intake");

        Color = hardwareMap.get(NormalizedColorSensor.class, "Color");
        ColorFront = hardwareMap.get(NormalizedColorSensor.class, "Color2");
        RampSensor = hardwareMap.get(NormalizedColorSensor.class, "RampSensor");


        setGrabber();
        CloseBox();
        IntakeBox();


    }

    private void SlidePower(double p){
        SlideR.setPower(-p);
        SlideL.setPower(p);
    }


    private void ReleasePixel() {
        PixelPusher.setPosition(0.15);
    }

    private void IntakePix() {
        Intake.setPower(1);
        IntakeRoller.setPower(-0.8);
    }

    private void OuttakePix() {
        Intake.setPower(-1);
        IntakeRoller.setPower(0.8);
    }

    private void CloseBox(){
        BucketHold.setPosition(0.7);
    }
    private void OpenBox(){
        //  BucketHold.setPosition(0.7); //close
        BucketHold.setPosition(0.5);


    }
    private void useGrabber(){
        Grabber.setPosition(0.85);
        GrabRoller.setPower(0.95);
    }

    private void setGrabber(){
        Grabber.setPosition(0.15);
        GrabRoller.setPower(0);
    }
    private void dropBox(){
        BucketHold.setPosition(0.05);
        sleep(100);
        BucketHold.setPosition(0.15);
    }//Bucket L Port 0
    //Bucket R Port 5
    //Bucket Hold Port 3

    private void BoardDropBox(){
        BucketL.setPosition(0.18);
        BucketR.setPosition(0.18);
    }
    public void stopRobot(){
        leftDrive.setPower(0);
        bleftDrive.setPower(0);
        rightDrive.setPower(0);
        brightDrive.setPower(0);
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
        BucketL.setPosition(0.5);
        BucketR.setPosition(0.5);
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


            if(x>=0 && x<=200){
                dropPos1 ++;
                telemetry.addData("Pixel Position :", "dropPos1");
            }
            if(x>=201 && x<=700){
                dropPos2 ++;
                telemetry.addData("Pixel Position:", "dropPos2");
            }
            if(x>=701 && x<=900){
                dropPos3 ++;
                telemetry.addData("Pixel Position:", "dropPos3");
            }
        }   // end for() loop

    }   // end method telemetryTfod()



}
