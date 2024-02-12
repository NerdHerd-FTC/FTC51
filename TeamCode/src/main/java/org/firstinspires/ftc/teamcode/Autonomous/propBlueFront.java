package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

@Autonomous(name="Blue Front")
public class propBlueFront extends lookForProp{
    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        initTfod();

        Pose2d spikeEndLocation = null;

        telemetry.addLine("redy to star, more like... you wount");
        telemetry.addData("tthe number thas the code:",314924.37771232333555);
        telemetry.update();

        waitForStart();

        //the robot is 8.775 in to its center from the front.
        //7.65 in from the side

        TrajectorySequence forward1 = drive.trajectorySequenceBuilder(new Pose2d(-39.55, 63.225, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-37.5, 54.52))
                .build();
        drive.setPoseEstimate(forward1.start());

        TrajectorySequence right = drive.trajectorySequenceBuilder(forward1.end())
                .splineToConstantHeading(new Vector2d(-46.35, 35.68), Math.toRadians(270.00))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(-36,33.545))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(-35.23, 37.39))
                .lineToConstantHeading(new Vector2d(-24.89, 35.68))
                .build();

        drive.followTrajectorySequence(forward1);

        String recognition = findProp(drive,"blue");

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
        telemetry.update();

        Trajectory back = drive.trajectoryBuilder(spikeEndLocation)
                .back(3)
                .build();

        TrajectorySequence returnToPosition = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(-36,38.68))
                .lineToConstantHeading(new Vector2d(-36,52.93))
                .build();

        TrajectorySequence leftBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(-29.92, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(26.90, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(45.19, 42.42), Math.toRadians(0.00))
                .build();

        TrajectorySequence centerBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(-29.92, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(26.90, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(45.19, 35.64), Math.toRadians(0.00))
                .build();

        TrajectorySequence rightBackdrop = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineTo(new Vector2d(-29.92, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(26.90, 59.22), Math.toRadians(0.00))
                .splineTo(new Vector2d(45.19, 29.05), Math.toRadians(0.00))
                .build();


// secret easy egg
        drive.followTrajectory(back);
        drive.followTrajectorySequence(returnToPosition);

        if (Objects.equals(recognition, "left")){
            drive.followTrajectorySequence(leftBackdrop);
        } else if (Objects.equals(recognition, "center")){
            drive.followTrajectorySequence(centerBackdrop);
        } else {
            drive.followTrajectorySequence(rightBackdrop);
        }
    }
}
