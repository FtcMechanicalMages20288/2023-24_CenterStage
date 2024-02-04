/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "TensyApril", group = "Auto")
public class AprilTensorflow extends LinearOpMode {


    int dropPos1;
    int dropPos2;
    int dropPos3;
    int finalDropPos;

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
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "BlueElement.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "TSE",
    };

    boolean aprilAdjust = true;





    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor  brightDrive = null;
    private DcMotor bleftDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private VisionPortal visionPortal1;

    @Override
    public void runOpMode() {


        initDoubleVision();
        sleep(3000);

        // Wait for the DS start button to be touched.
        //  telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        // telemetry.addData(">", "Touch Play to start OpMode");

        bleftDrive  = hardwareMap.get(DcMotor.class, "rm");
        brightDrive = hardwareMap.get(DcMotor.class, "lm");
        leftDrive  = hardwareMap.get(DcMotor.class, "brm");
        rightDrive = hardwareMap.get(DcMotor.class, "blm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.FORWARD);


        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);

        while (opModeInInit()) {
            telemetryTfod();
            telemetry.addData(" Pixel Position: ", finalDropPos);
            telemetry.update();
        }

        waitForStart();
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);



        sleep(3000);


        telemetry.addData("Final Pixel Position: ", finalDropPos);

        telemetry.update();
        if(finalDropPos == 1){


            while(aprilAdjust) {
                AprilTelemetry();

            }
           // telemetry.addData("Final Pixel Position: ", finalDropPos);

            //telemetry.update();

        //    driveForward(1.2);
        //    turnLeft(3);
        }
        else if(finalDropPos == 2){


            while(aprilAdjust) {
                AprilTelemetry();

            }
        //    telemetry.addData("Final Pixel Position: ", finalDropPos);

          //  telemetry.update();
        //    driveForward(1.2);

        }
        else if(finalDropPos ==3){




            while(aprilAdjust) {
                AprilTelemetry();

            }

        //    driveForward(1.2);
          //  turnRight(3);
        }







        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

    }   // end runOpMode()


    //Drive Methds

    private void driveForward(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        runtime.reset();
        // Set motor powers to drive forward
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        bleftDrive.setPower(FORWARD_SPEED);
        brightDrive.setPower(FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        bleftDrive.setPower(0);
        brightDrive.setPower(0);
    }

    private void turnRight(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        // Set motor powers to drive forward
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        bleftDrive.setPower(FORWARD_SPEED);
        brightDrive.setPower(-FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        bleftDrive.setPower(0);
        brightDrive.setPower(0);
    }

    private void turnLeft(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed


        // Set motor powers to drive forward
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        bleftDrive.setPower(-FORWARD_SPEED);
        brightDrive.setPower(FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        bleftDrive.setPower(0);
        brightDrive.setPower(0);
    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {



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
                .setModelLabels(LABELS)
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
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.

        builder.addProcessor(tfod);



        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()



                .build();

        aprilTag.setDecimation(2);


        VisionPortal.Builder builder1 = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder1.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder1.setCamera(BuiltinCameraDirection.BACK);
        }


        // Set and enable the processor.
        builder1.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal1 = builder1.build();

        visionPortal1.setProcessorEnabled(aprilTag, true);








    }

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
    private void AprilTelemetry() {
        double drive = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double turn = 0;
        targetFound = false;// Desired turning power/speed (-1 to +1) +ve is CounterClockwise
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        try {
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
                telemetry.addData("\n>", " Driving to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.update();
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                moveRobot(drive, turn);
                sleep(10);
            } else if (desiredTag.ftcPose.range - DESIRED_DISTANCE == DESIRED_DISTANCE) {
                aprilAdjust = false;
            } else {
                telemetry.addData("\n>", "No AprilTags found\n");
                telemetry.update();
            }

        } catch (NullPointerException e) {
            telemetry.addData("null", desiredTag);
            telemetry.update();
        }
    }


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
}   // end class
