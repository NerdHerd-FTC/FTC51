package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name="LookForPropTest")
public class lookForProp extends LinearOpMode {
    private static final String[] LABELS = {
            "blue",
            "red",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        initTfod();

        Trajectory forward1 = drive.trajectoryBuilder(new Pose2d(11.94, -63.13, Math.toRadians(90.00)))
                .lineTo(new Vector2d(11.94, -59.13))
                .build();
        drive.setPoseEstimate(forward1.start());


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(forward1);
        telemetry.addLine(findProp(drive,"red"));
        telemetry.update();
        sleep(1000);
    }

    public String findProp(org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive, String label) {
        if (getRecognition(label)){
            return "center";
        }
        drive.turn(Math.toRadians(30));
        if (getRecognition(label)){
            drive.turn(Math.toRadians(-30));
            return "left";
        }
        telemetry.update();
        drive.turn(Math.toRadians(-60));
        if (getRecognition(label)){
            drive.turn(Math.toRadians(30));
            return "right";
        }
        drive.turn(Math.toRadians(30));
        return "center"; //no detection, probably center
    }

    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("teamProp.tflite")
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public boolean getRecognition(String label){

        double currTime = getRuntime();
        double waitTime = currTime + (double)(1.5); //number is in seconds
        while (getRuntime() < waitTime){
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                if (Objects.equals(recognition.getLabel(), label)){
                    return true;
                }
            }
        }
        return false;
    }
}

