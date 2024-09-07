//package org.firstinspires.ftc.teamcode.drive;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.Barcode;
//import org.firstinspires.ftc.teamcode.drive.BlueScanner;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.List;
//
//
//@Autonomous(name = "OpenCV_RBR_World", group = "drive")
//public class OpenCV_RBR_World extends LinearOpMode {
//    private ElapsedTime runTime = new ElapsedTime();
//
//
//    OpenCvWebcam webcam;
//
//
//    private Servo BucketHold, BucketR, BucketL, PixelPusher;
//    private DcMotor SlideR, SlideL, Intake;
//    private CRServo GrabRoller;
//
//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
//    private DcMotor brightDrive = null;
//    private DcMotor bleftDrive = null;
//
//    private TouchSensor LimitSwitch;
//    private Servo Grabber;
//    private CRServo IntakeRoller;
//
//    public static double slidePower = 0.45;
//    int xValue = 19;
//    int yValue = -10;
//
//    public static int x2Value = 26;
//    public static int y2Value = 4 ;
//
//    public static int pos2startx = 30;
//    public static int pos2starty = 2 ;
//
//    public static int strafecorrect = 8 ;
//
//    public static int fwdDistancep1 = 30;
//    public static int fwdDistancep2 = 50;
//
//    public static int cycleX = 30;
//    public static int cycleY = 30;
//
//
//
//
//    public static double DISTANCE = 8; // in
//
//    public static int waitTime = 700;
//    public static int waitTimev2 = 750;
//    public static int waitTimev3 = 1000;
//
//    public static double FwBw = 8;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Status", "Run Time: " + runTime.toString());
//        telemetry.update();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        BlueScanner scanner = new BlueScanner(telemetry);
//        webcam.setPipeline(scanner);
//
//        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//            }
//        });
//
//        initCode();
//        CloseBox();
//
//        PixelPusher.setPosition(0.45); // locks pixel
//
//
//
//
//        Pose2d sP = new Pose2d(0,0,0);
//
//        drive.setPoseEstimate(sP);
//
//
//
//
//
//
//        Pose2d sP = new Pose2d(0,0,0);
//        Pose2d eel = drive.getPoseEstimate();
//        drive.setPoseEstimate(sP);
//
//
//
//
//        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(sP)
//
//                .lineToLinearHeading(new Pose2d(23, -13.3))
//                .addDisplacementMarker(() -> {
//                    ReleasePixel();
//                })
//                .addDisplacementMarker(() -> {
//                    sleep(500);
//                })
//                .back(9)
//                .strafeLeft(16.3)
//
//                /*
//                .lineToLinearHeading( new Pose2d(48, 0.5))
//                .turn(Math.toRadians(84))
//
//                .lineToConstantHeading(new Vector2d(55, 70))
//
//                 */
//                .lineToLinearHeading(new Pose2d(50, -2, Math.toRadians(90)))
//
//                //.turn(Math.toRadians(-88))
//                .lineToLinearHeading(new Pose2d(50, 70, Math.toRadians(90)))
//
//
//
//                .addDisplacementMarker(() -> {
//
//                    IntakeBox();
//                    SlidePower(slidePower);
//                    sleep(waitTime+500); //higher 500
//                    HoldSlides();
//                    BoardDropBox();
//                    // sleep(waitTimev2);
//
//                })
//                .lineTo(new Vector2d(54, 75))
//                /* .addDisplacementMarker(() -> {
//                     OpenBox();
//                     sleep(1000);
//                     backwardRobot();
//                     sleep(500);
//                     stopRobot();
//                 })
//                 .waitSeconds(1.6)*/
//                //.turn(Math.toRadians(200))
//                //.turn(Math.toRadians(-15.5))
//
//
//                .build();
//
//
//
//        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(sP)
//
//
//                .lineToLinearHeading(new Pose2d(20, 0))
//                .turn(Math.toRadians(50-11))
//                .forward(8)
//                .addDisplacementMarker(() -> {
//                    ReleasePixel();
//                })
//                .addDisplacementMarker(() -> {
//                    sleep(500);
//                })
//                .back(8)
//
//                //.lineTo(new Vector2d(18, 2))
//                .lineToLinearHeading(new Pose2d(50, -2, Math.toRadians(90)))
//
//                //.turn(Math.toRadians(-88))
//                .lineToLinearHeading(new Pose2d(50, 70, Math.toRadians(90)))
//
//                .addDisplacementMarker(() -> {
//
//                    IntakeBox();
//                    SlidePower(slidePower);
//                    sleep(waitTime+500);
//                    HoldSlides();
//                    BoardDropBox();
//                    // sleep(waitTimev2);
//
//                })
//                .lineTo(new Vector2d(54, 76.5))
//                /*.addDisplacementMarker(() -> {
//                    OpenBox();
//                    sleep(1000);
//                    backwardRobot();
//                    sleep(500);
//                    stopRobot();
//                })
//                .waitSeconds(1.6)*/
//                //.turn(Math.toRadians(200))
//                //.turn(Math.toRadians(-15.5))
//
//                .build();
//
//
//
//        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(sP)
//                .lineToLinearHeading(new Pose2d(pos2startx, pos2starty))
//                .addDisplacementMarker(() -> {
//                    ReleasePixel();
//                })
//                .addDisplacementMarker(() -> {
//                    sleep(500);
//                })
//                .back(4)
//                .turn(Math.toRadians(88))
//
//
//                .lineToLinearHeading(new Pose2d(26, 32, Math.toRadians(88)))
//                //Wait Adjust Variable. Default 0, bf strafe
//
//
//                .lineToLinearHeading(new Pose2d(25.5, 75.5, Math.toRadians(88)))
//                .addSpatialMarker(new Vector2d(25.7,73) , ()-> {
//                    IntakeBox();
//                    SlidePower(slidePower);
//                    sleep(waitTime+400);
//                    HoldSlides();
//                    BoardDropBox();
//                })
//
//                .waitSeconds(0.3)
//                .strafeRight(.5)
//
//                .forward(14,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//
//                )
//
//
//
//
//                .build();
//
//        TrajectorySequence pos1_park = drive.trajectorySequenceBuilder(pos1.end())
//                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
//                //.lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf
//                .build();
//
//        TrajectorySequence pos2_park = drive.trajectorySequenceBuilder(pos2.end())
//                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
//                // .lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf
//                .build();
//
//        TrajectorySequence pos3_park = drive.trajectorySequenceBuilder(pos3.end())
//                .lineToLinearHeading(new Pose2d(52, 80, Math.toRadians(90))) //X Changed 49 bf
//                //  .lineToLinearHeading(new Pose2d(52, 92, Math.toRadians(90))) //X Changed 49 bf
//                .build();
//
//
//        waitForStart();
//
//        runTime.reset();
//
//        Barcode result = scanner.getResult();
//
//
//
//        switch (result) {
//            case LEFT:
//                telemetry.addData("Detected", "Left");
//
//                drive.followTrajectorySequence(pos1);
//
//                //Wait Adjust Variable. Default 0, bf strafe
//
//                DESIRED_TAG_ID = 1;
//                strafeLeft();
//
//
//                while(aprilAdjust) {
//                    targetFound = false;
//                    desiredTag  = null;
//
//                    // Step through the list of detected tags and look for a matching tag
//                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//                    for (AprilTagDetection detection : currentDetections) {
//                        // Look to see if we have size info on this tag.
//                        if (detection.metadata != null) {
//                            //  Check to see if we want to track towards this tag.
//                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                                // Yes, we want to use this tag.
//                                targetFound = true;
//                                desiredTag = detection;
//                                break;  // don't look any further.
//                            } else {
//                                // This tag is in the library, but we do not want to track it right now.
//                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                            }
//                        } else {
//                            // This tag is NOT in the library, so we don't have enough information to track to it.
//                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                        }
//                    }
//
//                    // Tell the driver what we see, and what to do.
//                    if (targetFound) {
//
//                        strafeLeft();
//                        if(desiredTag.ftcPose.bearing < -8){
//                            aprilAdjust = false;
//                        }
//
//                    }
//                    telemetry.update();
//
//                    // Apply desired axes motions to the drivetrain.
//                    // moveRobot(drive, turn);
//                    sleep(10);
//
//
//                }
//
//
//                stopRobot();
//
//                forwardRobot();
//                sleep(600);
//                stopRobot();
//
//                OpenBox();
//                sleep(waitTimev3);
//
//                backwardRobot();
//                sleep(300);
//                stopRobot();
//                IntakeBox();
//
//                drive.followTrajectorySequence(pos1_park);
//
//            }
//
//            break;
//
//
//
//        // POS 2
//            case MIDDLE:
//                telemetry.addData("Detected", "Middle");
//
//                drive.followTrajectorySequence(pos2);
//
//                OpenBox();
//                sleep(1000);
//                backwardRobot();
//                sleep(500);
//                stopRobot();
//                IntakeBox();
//
//                drive.followTrajectorySequence(pos2_park);
//
//                break;
//
//
//            // POS 3
//            case RIGHT:
//                telemetry.addData("Detected", "Right");
//                drive.followTrajectorySequence(pos3);
//
//
//                DESIRED_TAG_ID = 3;
//                strafeLeft();
//
//
//                while(aprilAdjust) {
//                    targetFound = false;
//                    desiredTag  = null;
//
//                // Step through the list of detected tags and look for a matching tag
//                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//                for (AprilTagDetection detection : currentDetections) {
//                    // Look to see if we have size info on this tag.
//                    if (detection.metadata != null) {
//                        //  Check to see if we want to track towards this tag.
//                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                            // Yes, we want to use this tag.
//                            targetFound = true;
//                            desiredTag = detection;
//                            break;  // don't look any further.
//                        } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                        }
//                    } else {
//                        // This tag is NOT in the library, so we don't have enough information to track to it.
//                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                    }
//                }
//
//            // Tell the driver what we see, and what to do.
//                if (targetFound) {
//
//                    strafeLeft();
//                    if(desiredTag.ftcPose.bearing < -10){
//                        aprilAdjust = false;
//                    }
//
//                }
//            telemetry.update();
//
//            // Apply desired axes motions to the drivetrain.
//            // moveRobot(drive, turn);
//            sleep(10);
//
//
//            }
//
//            stopRobot();
//
//            forwardRobot();
//            sleep(1000);
//            stopRobot();
//
//            OpenBox();
//            sleep(waitTimev3);
//
//            backwardRobot();
//            sleep(300);
//            IntakeBox();
//            sleep(250);
//            stopRobot();
//
//            drive.followTrajectorySequence(pos3_park);
//
//
//
//            break;
//
//
//        }
//
//    }
//
//    private void SlideDown(){
//        boolean temp = true;
//
//        while(temp){
//            SlidePower(-0.4);
//            if(LimitSwitch.isPressed()){
//                telemetry.addData("Is Pressed", "HI");
//                telemetry.update();
//                temp = false;
//
//            }
//        }
//    }
//    private void initCode() {
//        SlideR = hardwareMap.dcMotor.get("SlideR");
//        SlideL = hardwareMap.dcMotor.get("SlideL");
//
//        bleftDrive = hardwareMap.get(DcMotor.class, "rm");
//        brightDrive = hardwareMap.get(DcMotor.class, "lm");
//        leftDrive = hardwareMap.get(DcMotor.class, "brm");
//        rightDrive = hardwareMap.get(DcMotor.class, "blm");
//
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//        brightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        PixelPusher = hardwareMap.get(Servo.class, "Pixel Pusher");
//
//        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
//
//        BucketL = hardwareMap.get(Servo.class, "BucketL");
//        BucketHold = hardwareMap.get(Servo.class, "BucketHold");
//        BucketR = hardwareMap.get(Servo.class, "BucketR");
//        BucketR.setDirection(Servo.Direction.REVERSE);
//
//        Grabber = hardwareMap.get(Servo.class, "Grab");
//        GrabRoller = hardwareMap.get(CRServo.class, "GrabRoller");
//        IntakeRoller = hardwareMap.get(CRServo.class, "Roll");
//        Intake = hardwareMap.dcMotor.get("Intake");
//
//
//        setGrabber();
//        CloseBox();
//        IntakeBox();
//
//
//    }
//
//    private void SlidePower(double p){
//        SlideR.setPower(-p);
//        SlideL.setPower(p);
//    }
//
//
//    private void ReleasePixel() {
//        PixelPusher.setPosition(0.15);
//    }
//
//    private void IntakePix() {
//        Intake.setPower(0.7);
//        IntakeRoller.setPower(-0.8);
//    }
//
//    private void OuttakePix() {
//        Intake.setPower(-1);
//        IntakeRoller.setPower(0.8);
//    }
//
//    private void CloseBox(){
//        BucketHold.setPosition(0.7);
//    }
//    private void OpenBox(){
//        //  BucketHold.setPosition(0.7); //close
//        BucketHold.setPosition(0.5);
//
//
//    }
//    private void useGrabber(){
//        Grabber.setPosition(0.85);
//        GrabRoller.setPower(0.95);
//    }
//
//    private void setGrabber(){
//        Grabber.setPosition(0.15);
//        GrabRoller.setPower(0);
//    }
//    private void dropBox(){
//        BucketHold.setPosition(0.05);
//        sleep(100);
//        BucketHold.setPosition(0.15);
//    }//Bucket L Port 0
//    //Bucket R Port 5
//    //Bucket Hold Port 3
//
//    private void BoardDropBox(){
//        BucketL.setPosition(0.955);
//        BucketR.setPosition(0.955);
//    }
//    public void stopRobot(){
//        leftDrive.setPower(0);
//        bleftDrive.setPower(0);
//        rightDrive.setPower(0);
//        brightDrive.setPower(0);
//    }
//
//    private void HoldSlides(){
//        SlideR.setPower(-0.1);
//        SlideL.setPower(0.1);
//    }
//
//    private void StopSlides(){
//        SlideR.setPower(0);
//        SlideL.setPower(0);
//    }
//    private void IntakeBox(){
//        BucketL.setPosition(0.63);
//        BucketR.setPosition(0.63);
////        BucketR.setPosition(0.59);
////        BucketL.setPosition(0.63);
//    }
//
//    public void strafeLeft(){
//        leftDrive.setPower(-0.52);
//        bleftDrive.setPower(0.5);
//        rightDrive.setPower(0.54);
//        brightDrive.setPower(-0.5);
//    }
//
//    public void forwardRobot(){
//        leftDrive.setPower(0.34);
//        bleftDrive.setPower(0.34);
//        rightDrive.setPower(0.34);
//        brightDrive.setPower(0.34);
//    }
//
//    public void backwardRobot(){
//
//        leftDrive.setPower(-0.34);
//        bleftDrive.setPower(-0.34);
//        rightDrive.setPower(-0.34);
//        brightDrive.setPower(-0.34);
//    }
//
//
//}