package org.firstinspires.ftc.teamcode.drive;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Barcode;
import org.firstinspires.ftc.teamcode.drive.BlueScanner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "OpenCV_RBL_World", group = "drive")
public class BlueAutoVision extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime();


    OpenCvWebcam webcam;


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

    boolean pixelCheck = false;
    boolean breakerCheck = false;





    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueScanner scanner = new BlueScanner(telemetry);
        webcam.setPipeline(scanner);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        initCode();
        CloseBox();

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
                .lineToLinearHeading(new Pose2d(x22Value - 1.25 ,y22Value,Math.toRadians(90)))


                .build();




        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(pos1.end())
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
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    SlideDown();

                })

                .lineToLinearHeading(new Pose2d(3.4,10,Math.toRadians(92))) //x = 4.5 prev
                .lineToLinearHeading(new Pose2d(3.4, -55, Math.toRadians(92))) //x = 4.5 prev
                .lineToLinearHeading(new Pose2d(15, -68, Math.toRadians(132-2)))
                //  .turn(Math.toRadians(-3.6))

                .back(6)

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

                .back(0.1)

                .waitSeconds(0.2)

                .addDisplacementMarker(() -> {

                    long timeOut = System.currentTimeMillis();

                    while (switcheroo && (timeOut + 2000) > (System.currentTimeMillis())) { // (< changed to > ) Loop while switcheroo is true and 5 seconds have not passed

                        if ((((DistanceSensor) RampSensor).getDistance(DistanceUnit.CM) < 4 && (((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2))) {
                           /* Intake.setPower(-1);
                            IntakeRoller.setPower(0.8); */
                            pixels++;
                            switcheroo = false;

                            telemetry.addData("RampSense: ", "yes");


                        }
                        else if((((DistanceSensor) Color).getDistance(DistanceUnit.CM) < 2) &&
                                (((DistanceSensor) ColorFront).getDistance(DistanceUnit.CM) < 1.5)){
                            pixels++;
                            switcheroo = false;
                            telemetry.addData("RampSense: ", "yes");
                        }
                        telemetry.update();
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
                    IntakePix();

                })
                // new code!!!!

                //.lineToLinearHeading(new Pose2d(15, -68, Math.toRadians(136)))
                .lineToLinearHeading(new Pose2d(3.4, -55, Math.toRadians(92)))
                .lineToLinearHeading(new Pose2d(3.4,10,Math.toRadians(92)))
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
                .lineToLinearHeading(new Pose2d(21,25, Math.toRadians(90)))
                .forward(FwBw+2,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )


                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(traj5p2.end())

                .back(FwBw)
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
                .lineToLinearHeading(new Pose2d(x33Value-1,y33Value+1,Math.toRadians(turn2+5))) //90
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
               // .forward(0.5)


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
                .lineToLinearHeading(new Pose2d(x44Value-3.5, y44Value,Math.toRadians(90)))
                .build();

        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(pos3.end())
                .forward(FwBw+3.5,
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


        TrajectorySequence traj3_2 = drive.trajectorySequenceBuilder(pos3.end())
                .forward(FwBw+2.5,
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


        TrajectorySequence breakPark = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(strafetox+5,strafetoy,Math.toRadians(90)))
                .build();

        TrajectorySequence tempPark = drive.trajectorySequenceBuilder(traj2.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(strafetox+8,33,Math.toRadians(-90)))
                .build();

        TrajectorySequence tempPark2 = drive.trajectorySequenceBuilder(traj2_2.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(strafetox + 8,33,Math.toRadians(-90)))
                .build();

        TrajectorySequence tempPark3 = drive.trajectorySequenceBuilder(traj2_3.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(strafetox + 8,33,Math.toRadians(-90)))
                .build();





        waitForStart();

        runTime.reset();

        Barcode result = scanner.getResult();



        switch (result) {
            case LEFT:
                telemetry.addData("Detected", "Left");

                drive.followTrajectorySequence(pos1);

                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();
                drive.followTrajectorySequence(traj2);



                //drive.followTrajectorySequence(traj3);

                /*
                drive.followTrajectorySequence(traj4);
                //useGrabber();


                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);

                pixelCheck = false;
                breakerCheck = false;
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
                sleep(waitTimev2);

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

                 */
                drive.followTrajectorySequence(tempPark);



                break;

            // POS 2
            case MIDDLE:
                telemetry.addData("Detected", "Middle");

                drive.followTrajectorySequence(pos2);

                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();
                drive.followTrajectorySequence(traj2_2);



                //drive.followTrajectorySequence(traj3);

                /*
                drive.followTrajectorySequence(traj4);
                //useGrabber();

                IntakePix();
                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);
                /*
                pixelCheck = false;
                breakerCheck = false;
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
                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime+200);
                HoldSlides();
                BoardDropBox();
                sleep(waitTimev2);

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

                 */
                drive.followTrajectorySequence(tempPark2);



                break;


            // POS 3
            case RIGHT:
                telemetry.addData("Detected", "Right");

                drive.followTrajectorySequence(pos3);
                IntakeBox();
                SlidePower(slidePower + 0.2);
                sleep(waitTime-250); //Change
                HoldSlides();
                BoardDropBox();




                drive.followTrajectorySequence(traj2_3);



                //drive.followTrajectorySequence(traj3);

                /*
                drive.followTrajectorySequence(traj4);
                //useGrabber();

                IntakePix();
                //sleep(waitTime+1500);


                drive.followTrajectorySequence(intake);


                drive.followTrajectorySequence(traj5);
                /*
                pixelCheck = false;
                breakerCheck = false;
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
                sleep(waitTimev2);

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
                */

                drive.followTrajectorySequence(tempPark3);



                break;


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
        Intake.setPower(0.7);
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
        BucketL.setPosition(0.955);
        BucketR.setPosition(0.955);
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
        BucketL.setPosition(0.63);
        BucketR.setPosition(0.63);
//        BucketR.setPosition(0.59);
//        BucketL.setPosition(0.63);
    }

}