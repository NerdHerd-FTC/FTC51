package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

@Autonomous(name="Red Front")
public class propRedFront extends lookForProp{
    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        DcMotor slideMotorR = hardwareMap.dcMotor.get("motorSlideR");
        DcMotor slideMotorL = hardwareMap.dcMotor.get("motorSlideL");
        Servo armTopServoR = hardwareMap.servo.get("armTopServoR");
        Servo armTopServoL = hardwareMap.servo.get("armTopServoL");

        initTfod();

        Pose2d spikeEndLocation = null;
        TrajectorySequence throughDoor = null;

        telemetry.addLine("redy to star, more like... you wount");
        telemetry.addData("tthe number thas the code:",314924.37771232333555);
        telemetry.update();

        waitForStart();

        //the robot is 8.775 in to its center from the front.
        //7.65 in from the side

        TrajectorySequence forward1 = drive.trajectorySequenceBuilder(new Pose2d(-39.55, -63.225, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-37.5, -54.52), slowedVelConst, slowedAccConst)
                .build();
        drive.setPoseEstimate(forward1.start());

        TrajectorySequence left = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(-35.23, -36.68), slowedVelConst, slowedAccConst)
                .lineToConstantHeading(new Vector2d(-44.35, -36.68))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(-36,-34.545))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(-35.23, -36.68), slowedVelConst, slowedAccConst)
                .lineToLinearHeading(new Pose2d(-28.3, -36.68,Math.toRadians(50)))
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
        telemetry.update();

        Trajectory back = drive.trajectoryBuilder(spikeEndLocation)
                .back(4)
                .build();

        TrajectorySequence returnToPosition = drive.trajectorySequenceBuilder(back.end())
                .lineToLinearHeading(new Pose2d(-36,-38.68,Math.toRadians(90)))
                .build();

        throughDoor = drive.trajectorySequenceBuilder(returnToPosition.end())
                .splineToSplineHeading(new Pose2d(-24.82, -12.46, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(31.36, -12.46), Math.toRadians(0.00))
                .build();

        if (Objects.equals(recognition,"center")){
            returnToPosition = drive.trajectorySequenceBuilder(back.end())
                    .lineToLinearHeading(new Pose2d(-36,-38.68,Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-36,-52.93))
                    .build();
            throughDoor = drive.trajectorySequenceBuilder(returnToPosition.end())
                    .splineToConstantHeading(new Vector2d(-57.03, -42.24), Math.toRadians(90)/*, org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint(40, org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL, org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH), org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)*/)
                    .splineToConstantHeading(new Vector2d(-57.69, -28.53), Math.toRadians(90.00))
                    .splineToLinearHeading(new Pose2d(-40.55, -11.04, Math.toRadians(0.00)), Math.toRadians(0.00))
                    .splineToConstantHeading(new Vector2d(29.77, -11.22), Math.toRadians(0.00))
                    .build();
        }




        TrajectorySequence rightBackdrop = drive.trajectorySequenceBuilder(throughDoor.end())
                .splineToConstantHeading(new Vector2d(45.19, -37.2), Math.toRadians(0.00))
                .build();

        TrajectorySequence centerBackdrop = drive.trajectorySequenceBuilder(throughDoor.end())
                .splineToConstantHeading(new Vector2d(45.19, -37.04), Math.toRadians(0.00))
                .build();

        TrajectorySequence leftBackdrop = drive.trajectorySequenceBuilder(throughDoor.end())
                .splineToConstantHeading(new Vector2d(45.19, -28.05), Math.toRadians(0.00))
                .build();


// secret easy egg
        drive.followTrajectory(back);
        drive.followTrajectorySequence(returnToPosition);
        drive.followTrajectorySequence(throughDoor);

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

        placePixel(drive,spikeEndLocation,slideMotorL,slideMotorR,armTopServoL,armTopServoR,"red");
    }
}
