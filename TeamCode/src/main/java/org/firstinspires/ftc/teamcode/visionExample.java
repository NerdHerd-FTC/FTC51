package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="vision example")
public class visionExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();

        //TfodProcessor tensorFlowProcessor;

        VisionPortal vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();

        waitForStart();

        List<AprilTagDetection> allTagDetections;  //create list of all detected apriltags
        int tagIdCode;

        // gets all detected tags
        allTagDetections = tagProcessor.getDetections();

        // process each tag in the list
        for (AprilTagDetection tagDetection : allTagDetections) {

            if (tagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                tagIdCode = tagDetection.id;

                // place any code based on detected id here

            }
        }
    }
}
