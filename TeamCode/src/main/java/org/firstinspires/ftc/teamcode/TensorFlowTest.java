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

import android.util.Log;
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/**
 * This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "TensorFlowTest", group = "Auto")
public class TensorFlowTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    int DropPos = 1;


    private static final String[] LABELS = {
            //"Ball",
            //"Cube",
            //"Duck",
            //"Marker"
            "TSE"
    };

    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;

    @Override
    public void runOpMode() {
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");

        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        initTfod();
        sleep(3000);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        int DropPos1 = 1;
        int DropPos2 = 1;
        int DropPos3 = 1;



        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        for (int j = 0; j < 10; j++) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel().equals("TSE")) {
                            int TSEpos = (int) recognition.getRight();
                            if (TSEpos <= 900 && TSEpos > 600) {
                                DropPos = 3;
                            } else if (TSEpos <= 600 && TSEpos >= 300) {
                                DropPos = 2;
                            } else if (TSEpos < 300 && TSEpos >= 0)
                                DropPos = 1;
                        } else {
                            DropPos = 2;
                        }
                    }

                    telemetry.addData("slidePos is", DropPos);
                    telemetry.update();

                    i++;
                }
                //Insert into the list

            }
        } // end of multiple detection for loop
        sleep(3000);


      /*  Log.d("FTC_20288", "Count all with frequency");
        Set<Integer> set = new HashSet<Integer>(DropPosList);
        /*for (Integer r : set) {
            Log.d("FTC_20288", (r + ": " + Collections.frequency(SlideLevelList, r)));
            int LevelCount = Collections.frequency(SlideLevelList, r);

        }
        int Level1Count = Collections.frequency(DropPosList, 1);
        int Level2Count = Collections.frequency(DropPosList, 2);
        int Level3Count = Collections.frequency(DropPosList, 3);
        Log.d("FTC_20288", (DropPos1 + ": " + DropPos2 + ": " + DropPos3));
        */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("slidePos is", DropPos);
        telemetry.update();
        waitForStart();
        telemetry.addData("Opmode started - Final slidePos is", DropPos);
        telemetry.update();

       if(DropPos == 2 ){
           driveEncoder(-31);
           //turnEncoder(20);
       }

        telemetry.update();
        sleep(3000);

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void driveEncoder(double distance){
        right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);



// manual set target position


        double circumference = 3.14 * 3.78;
        double rotationsNeeded;
        rotationsNeeded = -(distance / circumference);
        int encoderDrivingTarget = (int) (rotationsNeeded * 538);



        right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        left_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        back_left_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        back_right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));

        int right_front_pos = right_drive.getCurrentPosition();

        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        while(right_front_pos > encoderDrivingTarget) {
            right_drive.setPower(0.2);
            left_drive.setPower(0.2);
            back_left_drive.setPower(0.2);
            back_right_drive.setPower(0.2);

            right_front_pos = right_drive.getCurrentPosition();
            left_drive.getCurrentPosition();
            back_left_drive.getCurrentPosition();
            back_right_drive.getCurrentPosition();



            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.addData("encoderDrivingTarget: ", encoderDrivingTarget);
            telemetry.update();




            //turnEncoder(10,0.3);

        }


        right_drive.setPower(0);
        left_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        telemetry.addData("Trajectory","finished");
    }

    public void turnEncoder(double distance){
        right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_left_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        back_right_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);



// manual set target position


        double circumference = 3.14 * 3.78;
        double rotationsNeeded;
        rotationsNeeded = -(distance / circumference);
        int encoderDrivingTarget = (int) (rotationsNeeded * 538);



        right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));
        left_drive.setTargetPosition(encoderDrivingTarget);
        back_left_drive.setTargetPosition(encoderDrivingTarget);
        back_right_drive.setTargetPosition(Math.abs(encoderDrivingTarget));

        int right_front_pos = right_drive.getCurrentPosition();
        int left_front_pos = left_drive.getCurrentPosition();


        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        while(right_front_pos > encoderDrivingTarget && left_front_pos < encoderDrivingTarget ){
            right_drive.setPower(0.2);
            left_drive.setPower(-0.2);
            back_left_drive.setPower(-0.2);
            back_right_drive.setPower(0.2);

            right_front_pos = right_drive.getCurrentPosition();
            left_front_pos = left_drive.getCurrentPosition();

            left_drive.getCurrentPosition();
            back_left_drive.getCurrentPosition();
            back_right_drive.getCurrentPosition();



            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.addData("encoderDrivingTarget: ", encoderDrivingTarget);
            telemetry.update();




            //turnEncoder(10,0.3);

        }


        right_drive.setPower(0);
        left_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);

        telemetry.addData("Trajectory","finished");
    }


    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                //.setModelAssetName("RedElement.tflite")
                .setModelAssetName("BlueElement.tflite")

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
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
        //  builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

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
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
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
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
