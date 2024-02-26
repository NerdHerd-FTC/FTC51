package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

@Autonomous(name="Red Back")
public class propRedBack extends lookForProp{
    @Override
    public void runOpMode() {
        int WAIT_TIME = 0; //time to wait after starting, in ms.

        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        DcMotor slideMotorR = hardwareMap.dcMotor.get("motorSlideR");
        DcMotor slideMotorL = hardwareMap.dcMotor.get("motorSlideL");
        Servo armTopServoR = hardwareMap.servo.get("armTopServoR");
        Servo armTopServoL = hardwareMap.servo.get("armTopServoL");

        initTfod();

        Pose2d spikeEndLocation = null;

        telemetry.addLine("redy to star, more like... you wount");
        telemetry.addData("tthe number thas the code:",314924.37771232333555);
        telemetry.update();

        waitForStart();

        sleep(WAIT_TIME);

        //the robot is 8.775 in to its center from the front.
        //7.65 in from the side

        TrajectorySequence forward1 = drive.trajectorySequenceBuilder(new Pose2d(15.55, -63.225, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(13.5, -54.52), slowedVelConst, slowedAccConst)
                .build();
        drive.setPoseEstimate(forward1.start());

        TrajectorySequence right = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(11.23, -36.68), slowedVelConst, slowedAccConst)
                .lineToConstantHeading(new Vector2d(25.35, -36.68))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(12,-34.545))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(11.23, -36.68), slowedVelConst, slowedAccConst)
                .lineToLinearHeading(new Pose2d(6.39, -36.68,Math.toRadians(110)))
                .build();

        drive.followTrajectorySequence(forward1);

        String recognition = findProp(drive,"red");

        if (Objects.equals(recognition, "left")){
            drive.followTrajectorySequence(left);
            spikeEndLocation = left.end();
        } else if (Objects.equals(recognition, "center")){
            drive.followTrajectorySequence(center);
            spikeEndLocation = center.end();
        } else {
            drive.followTrajectorySequence(right);
            spikeEndLocation = right.end();
        }

        telemetry.addLine("Spike End Location X:"+spikeEndLocation.getX());
        telemetry.addLine("Spike End Location Y:"+spikeEndLocation.getY());
        telemetry.addLine("yersy");
        telemetry.update();

        Trajectory back = drive.trajectoryBuilder(spikeEndLocation)
                .back(5)
                .build();

        TrajectorySequence returnToPosition = drive.trajectorySequenceBuilder(back.end())
                .lineToLinearHeading(new Pose2d(12,-38.68,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(12,-52.93))
                .strafeRight(24)
                .build();

        TrajectorySequence rightBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(45.19, -41.95), Math.toRadians(0.00))
                .build();

        TrajectorySequence centerBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(45.19, -39.04), Math.toRadians(0.00))
                .build();

        TrajectorySequence leftBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(45.19, -33.05), Math.toRadians(0.00))
                .build();


// secret easy egg
        drive.followTrajectory(back);
        drive.followTrajectorySequence(returnToPosition);

        if (Objects.equals(recognition, "left")){
            drive.followTrajectorySequence(leftBackdrop);
            spikeEndLocation = leftBackdrop.end();
        } else if (Objects.equals(recognition, "center")){
            drive.followTrajectorySequence(centerBackdrop);
            spikeEndLocation = centerBackdrop.end();
        } else {
            drive.followTrajectorySequence(rightBackdrop);
            spikeEndLocation = rightBackdrop.end();
        }

        placePixel(drive,spikeEndLocation,slideMotorL,slideMotorR,armTopServoL,armTopServoR,"redB");
    }
}
