package org.firstinspires.ftc.teamcode.Examples.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Vision Driving - BLUE")
@Disabled
public class visionDrivingExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int navigateTo = 2; //set the tag number to navigate towards

        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");


        // Create apriltag processor
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        //TfodProcessor tensorFlowProcessor;

        // Add camera
        VisionPortal vPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();
        // Used to view what the robot sees on screen during init

        telemetry.addLine("Ready to start");
        telemetry.addLine();
        telemetry.addLine("Range is the distance to tag");
        telemetry.addLine("Bearing and Elevation are the angles to the tag");
        telemetry.addLine("idk man read the docs");
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

                tagIdCode = tagDetection.id; // Save current tag id to variable

                if (tagDetection.metadata != null && tagIdCode == navigateTo) {
                    /*
                    Checks if current tag is not null
                    Check is not necessary when only reading the tag id code
                    */

                    // place any code based on detected id here
                    // example code:

                    double tagRange = tagDetection.ftcPose.range; // Distance to tag (center)

                    // Angles the robot needs to turn to face the tag head on
                    double tagBearing = tagDetection.ftcPose.bearing; // left/right
                    double tagElevation = tagDetection.ftcPose.elevation; // up/down

                    if (tagBearing > 2) { //if the bearing is positive, turn counter-clockwise to the tag
                        frMotor.setPower(1);
                        flMotor.setPower(-1);
                        brMotor.setPower(1);
                        blMotor.setPower(-1);
                    } else if (tagBearing < -2) { //if the bearing is negative, turn clockwise to the tag
                        frMotor.setPower(-1);
                        flMotor.setPower(1);
                        brMotor.setPower(-1);
                        blMotor.setPower(1);
                    } else if (tagRange > 2){ //if neither is true, go straight. makes sure the robot doesn't ram into the tag
                        frMotor.setPower(1);
                        flMotor.setPower(1);
                        brMotor.setPower(1);
                        blMotor.setPower(1);
                    }

                    telemetry.addData("Tag "+ tagIdCode +" Range:", tagRange);
                    telemetry.addData("Tag "+ tagIdCode +" Bearing:", tagBearing);
                    telemetry.addData("Tag "+ tagIdCode +" Elevation:", tagElevation);
                } else{
                    //turns left if no tags are detected
                    frMotor.setPower(1);
                    flMotor.setPower(-1);
                    brMotor.setPower(1);
                    blMotor.setPower(-1);

                    telemetry.addLine("tag "+navigateTo+ " not detected");
                }
            }
            telemetry.update();
        }
    }
}
