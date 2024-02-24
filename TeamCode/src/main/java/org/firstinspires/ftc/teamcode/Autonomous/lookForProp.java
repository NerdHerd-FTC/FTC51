package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name="LookForPropTest")
@Disabled
public class lookForProp extends LinearOpMode {
    private static final String[] LABELS = {
            "blue",
            "red",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public TrajectoryVelocityConstraint slowedVelConst = org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint(13, org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL, org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH);
    public TrajectoryAccelerationConstraint slowedAccConst = SampleMecanumDrive.getAccelerationConstraint(20);
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
        drive.turn(Math.toRadians(-30));
        return "right"; //not center or left, probably right
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
        double waitTime = currTime + (double)(1); //number is in seconds
        while (getRuntime() < waitTime && opModeIsActive()){
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Current Recognitions", currentRecognitions);
            telemetry.update();
            for (Recognition recognition : currentRecognitions) {
                if (Objects.equals(recognition.getLabel(), label)){
                    return true;
                }
            }
        }
        return false;
    }

    public void placePixel(org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive, Pose2d startPosition, DcMotor slideMotorL, DcMotor slideMotorR, Servo armTopServoL, Servo armTopServoR, String direction){
        org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence strafeToSide;
        org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence pixelForward = drive.trajectorySequenceBuilder(startPosition)
                .waitSeconds(.5)
                .forward(6.25)
                .waitSeconds(.5)
                .back (5.25)
                .waitSeconds(1)
                .build();

        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        armTopServoR.setDirection(Servo.Direction.REVERSE);
        armTopServoL.setDirection(Servo.Direction.FORWARD);

        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setPower(1);
        slideMotorR.setPower(1);

        slideMotorL.setTargetPosition(150);
        slideMotorR.setTargetPosition(150);

        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armTopServoL.setPosition(.23);
        armTopServoR.setPosition(.23);

        drive.followTrajectorySequence(pixelForward);

        armTopServoL.setPosition(0);
        armTopServoR.setPosition(0);

        slideMotorL.setTargetPosition(0);
        slideMotorR.setTargetPosition(0);

        if (Objects.equals(direction, "blueB")) {
            strafeToSide = drive.trajectorySequenceBuilder(pixelForward.end())
                    .lineToConstantHeading(new Vector2d(pixelForward.end().getX(),58.63))
                    .build();
        } else if (Objects.equals(direction, "redB")){
            strafeToSide = drive.trajectorySequenceBuilder(pixelForward.end())
                    .lineToConstantHeading(new Vector2d(pixelForward.end().getX(),-58.63))
                    .build();
        } else if (Objects.equals (direction, "blueF")){
            strafeToSide = drive.trajectorySequenceBuilder(pixelForward.end())
                    .lineToConstantHeading(new Vector2d(pixelForward.end().getX(),12))
                    .build();
        } else{
            strafeToSide = drive.trajectorySequenceBuilder(pixelForward.end())
                    .lineToConstantHeading(new Vector2d(pixelForward.end().getX(),-12))
                    .build();
        }

        drive.followTrajectorySequence(strafeToSide);
    }
}

