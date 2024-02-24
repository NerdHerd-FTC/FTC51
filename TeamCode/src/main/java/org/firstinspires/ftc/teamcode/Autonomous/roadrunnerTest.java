package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RoadrunnerTest")
@Disabled
public class roadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);



        Trajectory blueClosetoPark = drive.trajectoryBuilder(new Pose2d(-36.40, 62.79, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-36.22, 27.54), Math.toRadians(-89.80))
                .splineTo(new Vector2d(6.64, 9.30), Math.toRadians(0.71))
                .splineTo(new Vector2d(52.34, 58.89), Math.toRadians(0.00))
                .build();

        drive.setPoseEstimate(blueClosetoPark.start());

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(blueClosetoPark);
    }
}

