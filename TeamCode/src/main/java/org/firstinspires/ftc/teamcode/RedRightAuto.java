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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RedRightAuto", group = "Auto")

public class RedRightAuto extends LinearOpMode {


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


    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor back_right_drive = null;
    private DcMotor back_left_drive = null;

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "lm");
        rightDrive = hardwareMap.get(DcMotor.class, "rm");
        back_right_drive = hardwareMap.get(DcMotor.class, "brm");
        back_left_drive = hardwareMap.get(DcMotor.class, "blm");



        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);

        //braking after each motion
        /*
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();



        //for(int i = 0; i< 10; i++) {
        //    telemetryTfod();
        //}

        /*telemetryTfod();

        // Push telemetry to the Driver Station.
        telemetry.update();

        if (dropPos1 > dropPos2 && dropPos1 > dropPos3) {
            finalDropPos = 1;

        } else if (dropPos2 > dropPos1 && dropPos2 > dropPos3) {
            finalDropPos = 2;
        } else if (dropPos3 > dropPos1 && dropPos3 > dropPos2) {
            finalDropPos = 3;
        } else {
            finalDropPos = 2;
            telemetry.addData("failed to find correct pixel pos: ", finalDropPos);
        }

        telemetry.addData("Final Pixel Position: ", finalDropPos);

        telemetry.update();*/

        while(opModeInInit()){

            telemetry.addData("DropPos1: ",dropPos1);
            telemetry.addData("DropPos2: ", dropPos2);
            telemetry.addData("DropPos3: ", dropPos3);
            telemetry.update();
        }
        waitForStart();



        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //AprilTelemetry();
                telemetryTfod();


                // Push telemetry to the Driver Station.
                telemetry.update();

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

                sleep(3000);

                if (finalDropPos == 1) {
                    telemetry.addData("Final Pixel Position: ", finalDropPos);

                    telemetry.update();

                    driveForward(1.1);
                    turnLeft(0.6);
                    driveForward(0.35);
                    //after deposition
                    driveBackward(0.55);
                    turnRight(0.6);
                    driveBackward(0.9);
                    // execute 90 degree turn
                    turnRight(1);
                    sleep(250);
                    driveForward(1.4);
                    // strafes to align with pixel board
                    // strafeRight(0.6);
                    // insert slide/bucket code to deposit pixel

                    telemetry.addData("Route Completed ", finalDropPos);
                    telemetry.update();

                    sleep(30000);


                } else if (finalDropPos == 2) {
                    telemetry.addData("Final Pixel Position: ", finalDropPos);

                    telemetry.update();
                    driveForward(1.35);
                    driveBackward(1);
                    // execute 90 degree turn
                    turnRight(1);
                    sleep(250);
                    driveForward(1.6);
                    // strafes to align with pixel board
                    // strafeRight(0.6);
                    // insert slide/bucket code to deposit pixel

                    telemetry.addData("Route Completed ", finalDropPos);
                    telemetry.update();

                    sleep(30000);


                } else if (finalDropPos == 3) {
                    telemetry.addData("Final Pixel Position: ", finalDropPos);

                    telemetry.update();

                    driveForward(1.1);
                    turnRight(0.6);
                    driveForward(0.35);
                    //after deposition
                    driveBackward(0.25);
                    turnLeft(0.6);
                    driveBackward(0.9);
                    // execute 90 degree turn
                    turnRight(1);
                    sleep(250);
                    driveForward(1.6);
                    // strafes to align with pixel board
                    // strafeRight(0.6);
                    // insert slide/bucket code to deposit pixel

                    telemetry.addData("Route Completed ", finalDropPos);
                    telemetry.update();

                    sleep(30000);
                }

                // requestOpModeStop();
            }
        }






        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    //Drive Methods

    private void strafeRight(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        runtime.reset();
        // Set motor powers to drive forward
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        back_left_drive.setPower(FORWARD_SPEED);
        back_right_drive.setPower(FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // tell motors to break when power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        sleep(850);
    }
    private void driveForward(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        runtime.reset();
        // Set motor powers to drive forward
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        back_left_drive.setPower(FORWARD_SPEED);
        back_right_drive.setPower(FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // tell motors to break when power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        sleep(850);

    }

    private void driveBackward(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        runtime.reset();
        // Set motor powers to drive forward
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        back_left_drive.setPower(-FORWARD_SPEED);
        back_right_drive.setPower(-FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // tell motors to break when power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        sleep(850);

    }

    private void turnLeft(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed

        // Set motor powers to drive forward
        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        back_left_drive.setPower(FORWARD_SPEED);
        back_right_drive.setPower(-FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // tell motors to break when power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        sleep(850);

    }

    private void turnRight(double seconds) {
        double FORWARD_SPEED = 0.4;  // Adjust as needed


        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        back_left_drive.setPower(-FORWARD_SPEED);
        back_right_drive.setPower(FORWARD_SPEED);

        // Reset runtime
        runtime.reset();

        // Continue driving until the specified duration is reached
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // You can add additional actions or conditions here if needed
        }

        // tell motors to break when power is zero
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        sleep(850);

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


            if(x>=0 && x<=350){
                dropPos1 ++;
                telemetry.addData("Pixel Position :", "dropPos1");
            }
            if(x>=351 && x<=890){
                dropPos2 ++;
                telemetry.addData("Pixel Position:", "dropPos2");
            }
            if(x>=891 && x<=900){
                dropPos3 ++;
                telemetry.addData("Pixel Position:", "dropPos3");
            }
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
