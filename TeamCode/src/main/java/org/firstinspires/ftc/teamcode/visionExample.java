package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="Vision Example")
public class visionExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Create apriltag processor
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        //TfodProcessor tensorFlowProcessor;

        // Add camera
        VisionPortal vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1440,960))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();
        // Used to view what the robot sees on screen

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        List<AprilTagDetection> allTagDetections;  /* create list of all detected apriltags
        List will be used to store any detected apriltags */

        int tagIdCode; // to store current apriltag id

        while (opModeIsActive()) {

            // Detect tags and save to list
            allTagDetections = tagProcessor.getDetections();

            // process each tag in the list
            for (AprilTagDetection tagDetection : allTagDetections) {

                if (tagDetection.metadata != null) {
                    /*
                    Checks if current tag is not null
                    Check is not necessary when only reading the tag id code
                    */

                    tagIdCode = tagDetection.id; // Save current tag id to variable

                    // place any code based on detected id here
                    // example code:

                    double tagRange = tagDetection.ftcPose.range;
                    double tagBearing = tagDetection.ftcPose.bearing;
                    double tagElevation = tagDetection.ftcPose.elevation;

                    telemetry.addData("Tag "+ tagIdCode +" Range:", tagRange);
                    telemetry.addData("Tag "+ tagIdCode +" Bearing:", tagBearing);
                    telemetry.addData("Tag "+ tagIdCode +" Elevation:", tagElevation);
                }
            }
            telemetry.update();
        }
    }
}
