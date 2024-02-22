package org.firstinspires.ftc.teamcode.Drivetrains;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

// UNUSED IN FAVOR OF MECANUM ROBOT ORIENTED

@TeleOp(name = "Mecanum - DO odometry")
@Disabled
public class mecanumDriveDO_odometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor leftEncoder = brMotor;
        DcMotor rightEncoder = blMotor;
        DcMotor frontEncoder = frMotor;

        // Turn off encoders for drivetrain
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double robotYaw = 0;
        double prevRight = 0;
        double prevLeft = 0;

        // Display controls
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Move (up, down, left, right)");
        telemetry.addLine("Right Stick- Turn");
        telemetry.addLine("Start - Reset Yaw");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;

            double leftDifference = prevLeft-leftEncoder.getCurrentPosition();
            double rightDifference = prevRight-rightEncoder.getCurrentPosition();
            robotYaw+=(leftDifference-rightDifference);
            prevLeft=leftEncoder.getCurrentPosition();
            prevRight=rightEncoder.getCurrentPosition();


            //this button should be hard to hit on accident
            //change if necessary
            if (gamepad1.start) {
                robotYaw = 0; // reset the yaw of the robot (obvious)
            }

            telemetry.addData("Robot Yaw",robotYaw);

            //calculates how much the robot should turn
            //directions are absolute
            //ex. if the stick is moved up, the robot will always move away from the drive team.
            double rotationX = stickX * Math.cos(-robotYaw) - stickY * Math.sin(-robotYaw);
            double rotationY = stickX * Math.sin(-robotYaw) + stickY * Math.cos(-robotYaw);


            //denominator is either the motor power or 1, depending on which is larger.
            //ensures all powers are the same ratio, but only when
            //at least one power is <-1 or >1
            double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(rStickX), 1);
            // Basically, it shrinks the movement amounts of each motor to fit in the range [-1,1]
            // The motors cannot move more then 1 or less then -1, so it is necessary to shrink the
            // movements amount to preserve the ratio between them all.
            // If no values are above 1, then denominator is 1 (no effect)

            // calculate how much each motor should move`
            double flPower = (rotationY + rotationX + rStickX) / denominator;
            double frPower = (rotationY - rotationX - rStickX) / denominator;
            double blPower = (rotationY - rotationX + rStickX) / denominator;
            double brPower = (rotationY + rotationX - rStickX) / denominator;

//            armControls(slideMotor,armTopServo,armRotateMotor,intakeMotor,droneServo);

            flMotor.setPower(flPower); // move the motors based on calculations
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);


            telemetry.update();
        }
    }
}
