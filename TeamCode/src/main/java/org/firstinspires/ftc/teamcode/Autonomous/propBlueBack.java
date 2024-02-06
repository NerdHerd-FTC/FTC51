package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Blue Back")
public class propBlueBack extends lookForProp{
    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        initTfod();

        Pose2d spikeEndLocation;

        waitForStart();

        Trajectory forward1 = drive.trajectoryBuilder(new Pose2d(11.05, 59.52, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(11.05, 54.52), Math.toRadians(270.00))
                .build();
        drive.setPoseEstimate(forward1.start());

        Trajectory left = drive.trajectoryBuilder(forward1.end())
                .splineToConstantHeading(new Vector2d(22.35, 32.68), Math.toRadians(270.00))
                .build();

        Trajectory center = drive.trajectoryBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(11.05,24.77))
                .build();

        Trajectory right = drive.trajectoryBuilder(forward1.end())
                .splineToConstantHeading(new Vector2d(11.23, 39.39), Math.toRadians(238.29))
                .splineToConstantHeading(new Vector2d(-0.89, 32.68), Math.toRadians(270.00))
                .build();

        drive.followTrajectory(forward1);

        String recognition = findProp(drive,"blue");

        if (recognition == "left"){
            drive.followTrajectory(left);
            spikeEndLocation = left.end();
        } else if (recognition == "center"){
            drive.followTrajectory(center);
            spikeEndLocation = center.end();
        } else {
            drive.followTrajectory(right);
            spikeEndLocation = right.end();
        }

        Trajectory back = drive.trajectoryBuilder(spikeEndLocation)
                .splineToConstantHeading(new Vector2d(spikeEndLocation.getX(), spikeEndLocation.getY()+3),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(11.05,45.18), Math.toRadians(270.0))
                .lineToConstantHeading(new Vector2d(11.05,52.93))
                .build();

        drive.followTrajectory(back);
    }
}
