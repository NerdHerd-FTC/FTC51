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

@TeleOp(name = "Mecanum - DO")
@Disabled
public class mecanumDrive extends armControls {
    @Override
    public void runOpMode() throws InterruptedException {
        //create objects for motors
        // f=front, b=back, l=left, r=right
        DcMotor flMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor frMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor blMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor brMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor slideMotor = hardwareMap.dcMotor.get("motorSlide");
        Servo armTopServo = hardwareMap.servo.get("armTopServo");
        DcMotor armRotateMotor = hardwareMap.dcMotor.get("armRotateMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo droneServo = hardwareMap.servo.get("droneServo");

        //reverse right side motors. reverse left side if goes backwards
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE); // IDK MAN, WE NEED TO TEST
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armTopServo.setDirection(Servo.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //gets the IMU (Inertial Measurement Unit) from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //sets orientation. change to match final robot
        //default is Logo Up and USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        // this is probably the issue


        imu.initialize(parameters);

        double servoPosition = 0.3;

        boolean droneLaunched = false;

        // Display controls
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Move (up, down, left, right)");
        telemetry.addLine("Right Stick- Turn");
        telemetry.addLine("Start - Reset Yaw");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Forward Position (150 Deg)");
        telemetry.addLine("LB - Rotate Arm Servo to 0 Degrees");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Forwards");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Backwards");
        telemetry.addLine("A button - Toggle intake on/off");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();

        boolean intakeButtonPressed = false;

        while (opModeIsActive()) {
            //gamepad variables
            double stickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double stickX = gamepad1.left_stick_x;
            double rStickX = gamepad1.right_stick_x;



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
