package org.firstinspires.ftc.teamcode;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Mecanum - DO")
public class mecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //gets the IMU (Inertial Measurement Unit) from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //sets orientation. change to match final robot
        //default is Logo Up and USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Move");
        telemetry.addLine("Right Stick- Strafe");
        telemetry.addLine("Start - Reset Yaw");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
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

            double rTrigger = gamepad1.right_trigger;
            double lTrigger = gamepad1.left_trigger;

            //this button should be hard to hit on accident
            //change if necessary
            if (gamepad1.start) {
                imu.resetYaw(); // reset the yaw of the robot (obvious)
            }



            //retrieves the yaw of the robot
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Robot Yaw",robotYaw);

            //calculates how much the robot should turn
            //directions are absolute
            //ex. if the stick is moved up, the robot will always move away from the drive team.
            double rotationX = stickX * Math.cos(-robotYaw) - stickY * Math.sin(-robotYaw);
            double rotationY = stickX * Math.sin(-robotYaw) + stickY * Math.cos(-robotYaw);

            rotationX = rotationX * 1.1; //counteract imperfect strafing

            //denominator is either the motor power or 1, depending on which is larger.
            //ensures all powers are the same ratio, but only when
            //at least one power is <-1 or >1
            double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(rStickX), 1);
            // Basically, it shrinks the movement amounts of each motor to fit in the range [-1,1]
            // The motors cannot move more then 1 or less then -1, so it is necessary to shrink the
            // movements amount to preserve the ratio between them all.
            // If no values are above 1, then denominator is 1 (no effect)

            // calculate how much each motor should move
            double flPower = (rotationY + rotationX + rStickX) / denominator;
            double frPower = (rotationY - rotationX - rStickX) / denominator;
            double blPower = (rotationY - rotationX + rStickX) / denominator;
            double brPower = (rotationY + rotationX - rStickX) / denominator;

            flMotor.setPower(flPower); // move the motors based on calculations
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);

            slideMotor.setPower(rTrigger-lTrigger);

            telemetry.update();
        }
    }
}
