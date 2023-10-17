package org.firstinspires.ftc.teamcode;

//javadocs here: https://javadoc.io/doc/org.firstinspires.ftc
//ftc docs here: https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Drive train controls for mecanum drive
// Mecanum drive allows omnidirectional movement

@TeleOp(name = "Tank")
public class tankDriveArm extends LinearOpMode {
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

//        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        armRotateMotor.setTargetPosition(0);

//        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean droneLaunched = false;

        // Display controls
        telemetry.addLine("Variables initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick - Left Side Wheels");
        telemetry.addLine("Right Stick - Right Side Wheels");
        telemetry.addLine("Start - Reset Yaw");
        telemetry.addLine("RT - Extend Arm");
        telemetry.addLine("LT - Retract Arm");
        telemetry.addLine("RB - Rotate Arm Servo to Forward Position (150 Deg)");
        telemetry.addLine("LB - Rotate Arm Servo to 0 Degrees");
        telemetry.addLine("D-Pad Up - Rotate Arm Body Forwards");
        telemetry.addLine("D-Pad Down - Rotate Arm Body Backwards");
        telemetry.addLine("A button - Toggle intake on/off");
        telemetry.addLine("Back - Launch Drone");
        telemetry.addLine();
        telemetry.addLine("Ready to start");
        telemetry.update();

        //waits for start of game
        waitForStart();
        droneServo.setPosition(0);

        boolean intakeButtonPressed = false;

        while (opModeIsActive()) {
            //gamepad variables
            double lstickY = -gamepad1.left_stick_y; //Y stick value is REVERSED
            double rStickY = gamepad1.right_stick_y;

            //triggers for arm extending
            double rTrigger = gamepad1.right_trigger;
            double lTrigger = gamepad1.left_trigger;

            //in our case, our servos have a range of 300 degrees
            //the position numbers are a fraction of these 300 degrees
            // The position goes from 0 to 1
            // This represents the fraction of 300 degrees the motor should be at
            // eg. 0.5 would be 150 degrees & 0.1 would be 30.
            if (gamepad1.right_bumper){
                armTopServo.setPosition(0.6);
            } else if (gamepad1.left_bumper) {
                armTopServo.setPosition(0.125);
            }

            if (gamepad1.a && !intakeButtonPressed) { // Toggle intake motor on/off
                intakeMotor.setPower(1-intakeMotor.getPower());
                telemetry.addLine("Intake is moving");
                intakeButtonPressed = true;
            } else {
                telemetry.addLine("Intake is stopped");
            }


            // controls to rotate the whole arm up and down (forwards and backwards)
            if (gamepad1.dpad_up) {
                armRotateMotor.setPower(.7); //makes the arm motors rotate forwards slowly
                telemetry.addLine("Moving arm forward");
            } else if (gamepad1.dpad_down) {
                armRotateMotor.setPower(-.7); //makes the arm motors rotate backwards slowly
                telemetry.addLine("Moving arm backward");
            } else {
                telemetry.addLine("Arm isn't moving");
            }

            // only changes position when the motor isn't busy, (hopefully) making controls more precise

            // 5700.4 counts per revolution
            /*
            if (gamepad1.dpad_up && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + 50); //makes the arm motors rotate forwards slowly
            } else if (gamepad1.dpad_down && !armRotateMotor.isBusy()) {
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - 50); //makes the arm motors rotate backwards slowly
            }// TODO: add telemetry
             */


            //moves the drone servo to the launch position
            if (gamepad1.back) {
                droneServo.setPosition(1);
            }
            telemetry.addData("Drone Launched",droneLaunched);

            flMotor.setPower(lstickY);
            blMotor.setPower(lstickY);

            frMotor.setPower(rStickY);
            brMotor.setPower(rStickY);

            slideMotor.setPower(rTrigger-lTrigger+0.05); // move slide motor
            // the 0.05 is to counteract gravity
            // telemetry.addData("current arm motion:",rTrigger-lTrigger);
            telemetry.update();
        }
    }
}
