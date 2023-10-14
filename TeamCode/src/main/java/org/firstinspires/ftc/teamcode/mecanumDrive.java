package org.firstinspires.ftc.teamcode;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html
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

        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //gets the IMU (Inertial Measurement Unit) from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //sets orientation. change to match final robot
        //default is Logo Up and USB Forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

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
        telemetry.addLine("X button - Toggle intake on/off");
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

            //triggers for arm extending
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

//            rotationX = rotationX * 1.1; //counteract imperfect strafing


            //in our case, our servos have a range of 300 degrees
            //the position numbers are a fraction of these 300 degrees
            // The position goes from 0 to 1
            // This represents the fraction of 300 degrees the motor should be at
            // eg. 0.5 would be 150 degrees & 0.1 would be 30.
            if (gamepad1.right_bumper){
                servoPosition=0.5;
            } else if (gamepad1.left_bumper) {
                servoPosition=0;
            }
            telemetry.addData("Arm is up",(servoPosition>=0.4));

            if (gamepad1.x && !intakeButtonPressed) { // Toggle intake motor on/off
                intakeMotor.setPower(1-intakeMotor.getPower());
                telemetry.addLine("Intake is moving");
                intakeButtonPressed = true;
            } else {
                telemetry.addLine("Intake is stopped");
            }


            // controls to rotate the whole arm up and down (forwards and backwards)
            if (gamepad1.dpad_up) {
                armRotateMotor.setPower(1); //makes the arm motors rotate forwards slowly
            } else if (gamepad1.dpad_down) {
                armRotateMotor.setPower(-1); //makes the arm motors rotate backwards slowly
            } else {
                armRotateMotor.setPower(0);
            }// TODO: add telemetry

            //moves the drone servo to the launch position
            if (gamepad1.back) {
                droneServo.setPosition(0);
            }
            telemetry.addData("Drone Launched",droneLaunched);

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

            flMotor.setPower(flPower); // move the motors based on calculations
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);

            slideMotor.setPower(rTrigger-lTrigger+0.05); // move slide motor
            // the 0.05 is to counteract gravity
            // telemetry.addData("current arm motion:",rTrigger-lTrigger);

            armTopServo.setPosition(servoPosition);
            telemetry.addData("Arm Servo Position", servoPosition);

            telemetry.update();
        }
    }
}
